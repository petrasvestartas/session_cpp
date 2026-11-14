#include "brep_boolean.h"
#include "brep_assemble.h"
#include "brep_classify.h"
#include "brep_section.h"
#include "brep_interference_internal.h"
#include "curve_intersection.h"
#include "curve_surface_intersection.h"
#include "split.h"
#include "nurbs_bounds.h"
#include "nurbs_coincidence.h"
#include "nurbs_memo.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <numeric>
#include <fstream>
#include <stdexcept>

namespace session_cpp::brep_boolean {
namespace {

using State = brep_classify::State;

void boolean_progress(const char* phase) {
    if (!std::getenv("SESSION_BOOLEAN_PROGRESS"))
        return;
    std::fprintf(stderr, "boolean_progress phase=%s\n", phase);
    std::fflush(stderr);
}

struct Partition {
    BRep brep;
    std::vector<split::EdgePiece> pieces;
    int operand;
    std::vector<std::vector<int>> node_vertices; // The vertices of the model each interference node lies on.
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

/// Ends of two cuts within the model tolerance are one node: sections meeting on a third face's edge end a march step apart, and the conservative uv tolerance would leave the chain open and drop the whole cut as dangling. Ends a seam apart are one point in space but stay two.
void join_cut_ends(std::vector<split::FaceCut>& cuts, const BRep& brep, int face, double tolerance) {
    const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
    // Ends a rounding apart are already one point; a hair further and the arrangement would find one cut ending on the other beside its end, a piece inside the vertex.
    const std::pair<double, double> span_u = surface.domain(0), span_v = surface.domain(1);
    const double merged = 256.0 * std::numeric_limits<double>::epsilon() * std::max({1.0, std::abs(span_u.first),
        std::abs(span_u.second), std::abs(span_v.first), std::abs(span_v.second)});
    const double reach = 64.0 * uv_tolerance(surface, tolerance);
    const auto cv = [](const split::FaceCut& cut, int end) { return end ? cut.pcurve.cv_count() - 1 : 0; };
    const auto world = [](const split::FaceCut& cut, int end) {
        return end ? cut.world.point_at_end() : cut.world.point_at_start();
    };
    // The end of a clamped curve is its end control point whatever its weight, which stays.
    const auto move_end = [&cv](split::FaceCut& cut, int end, const Point& uv) {
        const int index = cv(cut, end);
        const double weight = cut.pcurve.is_rational() ? cut.pcurve.weight(index) : 1.0;
        cut.pcurve.set_cv_4d(index, uv[0] * weight, uv[1] * weight, uv[2] * weight, weight);
    };
    for (size_t i = 0; i < cuts.size(); ++i)
        for (size_t j = i + 1; j < cuts.size(); ++j)
            for (int a = 0; a < 2; ++a)
                for (int b = 0; b < 2; ++b) {
                    if (!cuts[i].pcurve.is_clamped() || !cuts[j].pcurve.is_clamped())
                        continue;
                    const Point first = cuts[i].pcurve.get_cv(cv(cuts[i], a));
                    const Point second = cuts[j].pcurve.get_cv(cv(cuts[j], b));
                    const double gap = first.distance(second);
                    if (gap <= merged || gap > reach || world(cuts[i], a).distance(world(cuts[j], b)) > tolerance)
                        continue;
                    const Point middle((first[0] + second[0]) * 0.5, (first[1] + second[1]) * 0.5, (first[2] + second[2]) * 0.5);
                    move_end(cuts[i], a, middle);
                    move_end(cuts[j], b, middle);
                }
    // The same for an end at a trim vertex: left a hair off it, the cut would re-split the edge beside the vertex.
    for (split::FaceCut& cut : cuts)
        for (int end = 0; end < 2 && cut.pcurve.is_clamped(); ++end)
            for (const BRepRef& wire : brep.m_faces[face].wires)
                for (const BRepRef& edge : brep.wire_edges(wire)) {
                    const int index = brep.pcurve_index(edge.index, face, edge.orientation);
                    if (index < 0)
                        continue;
                    const NurbsCurve& trim = brep.m_curves_2d[index];
                    for (const Point& corner : {trim.point_at_start(), trim.point_at_end()}) {
                        const double gap = cut.pcurve.get_cv(cv(cut, end)).distance(corner);
                        if (gap > merged && gap <= reach &&
                            world(cut, end).distance(surface.point_at(corner[0], corner[1])) <= tolerance)
                            move_end(cut, end, corner);
                    }
                }
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

/// Record the ancestry of the edges a split added to the partition's model.
void retain_lineage(Partition& result, const std::vector<split::EdgePiece>& edges) {
    const size_t old_size = result.pieces.size();
    result.pieces.resize(result.brep.m_edges.size());
    for (size_t i = old_size; i < result.pieces.size(); ++i)
        result.pieces[i].edge = static_cast<int>(i);
    for (split::EdgePiece piece : edges) {
        if (piece.source_edge >= 0) {
            if (piece.source_edge >= static_cast<int>(old_size))
                throw std::runtime_error("Native Boolean split has an unresolved parent edge");
            const split::EdgePiece& parent = result.pieces[piece.source_edge];
            piece.source_edge = parent.source_edge;
            piece.section = parent.section;
        }
        result.pieces[piece.edge] = piece;
    }
}

void retain_lineage(Partition& result, split::FaceSplit next) {
    result.brep = std::move(next.brep);
    retain_lineage(result, next.edges);
}

// Canonical graph nodes are materialized separately in each existing topological
// component. Distinct original vertex identities are never proximity-welded.
bool split_original_events(Partition& partition,const brep_section::FaceResult& inventory,
                           bool second,double tolerance) {
    const auto component=brep_section::detail::active_components(partition.brep,active_faces(partition.brep));
    const auto root=[&](int vertex) { return component[vertex]; };
    const int original_vertices=static_cast<int>(partition.brep.m_vertices.size());
    std::vector<split::EdgeEvent> events;
    partition.node_vertices.assign(inventory.nodes.size(),{});
    for (size_t node_index=0;node_index<inventory.nodes.size();++node_index) {
        const auto& node=inventory.nodes[node_index];
        struct LocalNode {
            int vertex=-1;
            std::vector<const brep_section::BoundaryEvent*> events;
        };
        std::map<int,LocalNode> groups;
        const auto retain_vertex=[&](int vertex) {
            auto& local=groups[root(vertex)];
            if (local.vertex>=0 && local.vertex!=vertex)
                throw std::runtime_error("Interference node has distinct existing owner-local vertices");
            local.vertex=vertex;
        };
        for (size_t index:node.uv_only_incidences) {
            const auto& use=inventory.uv_only_incidences[index];
            if (use.operand==static_cast<int>(second))
                retain_vertex(use.vertex);
        }
        for (size_t index:node.boundary_events) {
            const auto& event=inventory.boundary_events[index];
            if (event.operand==static_cast<int>(second)) {
                const auto& edge=partition.brep.m_edges[event.edge];
                groups[root(edge.start_vertex)].events.push_back(&event);
                if (event.vertex>=0)
                    retain_vertex(event.vertex);
            }
        }
        for (auto& [unused,local]:groups) {
            // UV-only incidences preserve their existing vertex without a fake raw t.
            if (local.vertex<0 && !local.events.empty())
                local.vertex=partition.brep.add_vertex(node.point,tolerance);
            if (local.vertex>=0)
                partition.node_vertices[node_index].push_back(local.vertex);
            for (const auto* event:local.events)
                events.push_back({event->edge,event->hit.t,local.vertex});
        }
    }
    // Two nodes on one edge within the model tolerance are one event: sections ending a march step apart would otherwise cut a piece the other operand has no partner for.
    std::sort(events.begin(),events.end(),[](const split::EdgeEvent& x,const split::EdgeEvent& y) {
        return x.edge!=y.edge ? x.edge<y.edge : x.parameter<y.parameter;
    });
    std::vector<split::EdgeEvent> joined;
    std::vector<std::pair<int,int>> merged;
    for (const auto& event:events) {
        if (!joined.empty() && joined.back().edge==event.edge && !partition.brep.m_edges[event.edge].degenerated) {
            const NurbsCurve& curve=partition.brep.m_curves_3d[partition.brep.m_edges[event.edge].curve_3d_index];
            if (curve.point_at(joined.back().parameter).distance(curve.point_at(event.parameter))<=tolerance) {
                split::EdgeEvent& kept=joined.back();
                const int other=kept.vertex;
                if (event.vertex<original_vertices && kept.vertex>=original_vertices)
                    kept=event;
                else if (event.vertex>=original_vertices && kept.vertex>=original_vertices)
                    kept.parameter=(kept.parameter+event.parameter)*0.5;
                merged.push_back({kept.vertex==other?event.vertex:other,kept.vertex});
                continue;
            }
        }
        joined.push_back(event);
    }
    events=std::move(joined);
    // A node whose event was joined to another's on one edge lies on that vertex there.
    for (auto& vertices:partition.node_vertices)
        for (const auto& [dropped,kept]:merged)
            if (dropped!=kept && std::find(vertices.begin(),vertices.end(),dropped)!=vertices.end() &&
                std::find(vertices.begin(),vertices.end(),kept)==vertices.end())
                vertices.push_back(kept);
    if (!events.empty()) {
        auto partitioned=split::split_brep_edges_at_events(partition.brep,events,tolerance);
        for (const auto& event:events)
            if (std::none_of(partitioned.edge_events.begin(),partitioned.edge_events.end(),[&](const auto& receipt) {
                    return receipt.edge==event.edge && receipt.parameter==event.parameter && receipt.vertex==event.vertex;
                }))
                throw std::runtime_error("Original edge event receipt lost its canonical owner-local vertex");
        retain_lineage(partition,std::move(partitioned));
    }
    return !events.empty();
}

void observe_partition(const char* path, const BRep& source, const std::vector<split::FaceCut>& cuts,
                       const split::ArrangementDiagnostic& diagnostic, double tolerance, const char* stage, int operand = 0, int face = 5) {
    nlohmann::ordered_json row = {{"operand", operand}, {"face", face}, {"stage", stage},
        {"tolerance", tolerance}, {"shape", source.jsondump()},
        {"cuts", nlohmann::ordered_json::array()}, {"boundary_cut_pairs", nlohmann::ordered_json::array()},
        {"uv_tolerance", diagnostic.uv_tolerance}, {"sources", diagnostic.sources},
        {"spans", diagnostic.spans}, {"nodes", diagnostic.nodes}, {"half_edges", diagnostic.half_edges},
        {"retained_cycles", diagnostic.retained_cycles}, {"regions", diagnostic.regions}};
    for (const auto& cut : cuts)
        row["cuts"].push_back({{"world", cut.world.jsondump()}, {"pcurve", cut.pcurve.jsondump()}, {"section", cut.section}});
    for (const auto& pair : diagnostic.boundary_cut_pairs)
        row["boundary_cut_pairs"].push_back({{"first_source", pair.first_source}, {"second_source", pair.second_source},
            {"first", pair.first.jsondump()}, {"second", pair.second.jsondump()},
            {"first_hits", pair.first_hits}, {"second_hits", pair.second_hits}, {"overlaps", pair.overlaps}});
    row["cycles"] = nlohmann::ordered_json::array();
    for (const auto& cycle : diagnostic.cycles) {
        nlohmann::ordered_json points = nlohmann::ordered_json::array();
        for (const auto& p : cycle.points) points.push_back({p[0], p[1], p[2]});
        row["cycles"].push_back({{"area", cycle.area}, {"points", points}, {"edge_ids", cycle.edges},
            {"disposition", cycle.disposition}, {"runs", cycle.runs}, {"first_source", cycle.first_source},
            {"probe_evaluated", cycle.probe_evaluated}, {"probe", {cycle.probe[0], cycle.probe[1], cycle.probe[2]}}});
    }
    row["parent_tests"] = nlohmann::ordered_json::array();
    for (const auto& test : diagnostic.parent_tests)
        row["parent_tests"].push_back({{"hole_retained_index", test.hole}, {"outer_retained_index", test.outer},
            {"shared_edge", test.shared}, {"area_eligible", test.area_eligible},
            {"inside_evaluated", test.inside_evaluated}, {"inside", test.contained}});
    std::ofstream output(path, std::ios::app);
    if (!output)
        throw std::runtime_error(std::string("Cannot open dodecahedron partition observation: ") + path);
    output << row.dump() << '\n';
    output.flush();
    if (!output)
        throw std::runtime_error(std::string("Cannot write dodecahedron partition observation: ") + path);
}

/// Name the ends of a section curve strictly inside its faces for the face split: the boundary vertex its node became, or the node itself where the end is inside the face.
void place_cut(split::FaceCut& cut, const brep_section::FaceCurve& section,
               const std::vector<std::vector<int>>& node_vertices, const std::vector<int>& corners) {
    cut.placed = (section.nodes[0] < 0) == (section.nodes[1] < 0);
    for (int end = 0; end < 2 && cut.placed; ++end) {
        if (section.nodes[end] < 0)
            continue;
        // A joint of two section curves is no node of an edge: it has no vertex.
        for (size_t i = 0; section.nodes[end] < static_cast<int>(node_vertices.size()) && i < node_vertices[section.nodes[end]].size(); ++i) {
            const int vertex = node_vertices[section.nodes[end]][i];
            if (std::find(corners.begin(), corners.end(), vertex) != corners.end()) {
                cut.placed = cut.placed && (cut.vertices[end] < 0 || cut.vertices[end] == vertex);
                cut.vertices[end] = vertex;
            }
        }
        if (cut.vertices[end] < 0)
            cut.nodes[end] = section.nodes[end];
    }
}

/// Mark the edges of a partition that their source and range alone place in an assembly: section curves strictly inside both faces, and original edges that cross the other operand's faces strictly inside them, with no edge of it along them or at their nodes.
void mark_exact(Partition& result, const BRep& source, const brep_section::FaceResult& sections, bool second) {
    std::vector<bool> met(source.edge_count(), false);
    for (int edge : sections.overlapping_edges[second])
        met[edge] = true;
    for (const brep_section::PointNode& node : sections.nodes) {
        bool other = !node.support_points.empty();
        for (size_t index : node.boundary_events)
            other = other || sections.boundary_events[index].operand != static_cast<int>(second) || !sections.boundary_events[index].inside;
        for (size_t index : node.boundary_events)
            if (other && sections.boundary_events[index].operand == static_cast<int>(second))
                met[sections.boundary_events[index].edge] = true;
    }
    for (split::EdgePiece& piece : result.pieces)
        piece.exact = piece.section >= 0 ? sections.curves[piece.section].interior
            : piece.source_edge >= 0 && !met[piece.source_edge];
}

Partition partition(
    const BRep& source,
    const BRep& other,
    const brep_section::FaceResult& sections,
    bool second,
    double tolerance
) {
    Partition result{source, {}, second ? 1 : 0, {}};
    for (int i = 0; i < source.edge_count(); ++i) {
        const BRepEdge& edge = source.m_edges[i];
        const std::pair<double, double> domain = edge.degenerated ? std::pair<double, double>{0, 1}
            : source.m_curves_3d[edge.curve_3d_index].domain();
        result.pieces.push_back({i, -1, i, domain.first, domain.second});
    }
    // The edge split certifies the whole model; without one the first face split does, and the later ones only what they make.
    bool validated = split_original_events(result,sections,second,tolerance);
    for (int face : active_faces(source)) {
        std::vector<split::FaceCut> cuts;
        std::vector<int> corners;
        for (const BRepRef& wire : result.brep.m_faces[face].wires)
            for (const BRepRef& edge : result.brep.wire_edges(wire))
                corners.insert(corners.end(), {result.brep.m_edges[edge.index].start_vertex, result.brep.m_edges[edge.index].end_vertex});
        for (size_t i = 0; i < sections.curves.size(); ++i) {
            const brep_section::FaceCurve& section = sections.curves[i];
            if (section.touching || (second ? section.second_face : section.first_face) != face)
                continue;
            split::FaceCut cut{section.world, second ? section.second_pcurve : section.first_pcurve, static_cast<int>(i)};
            if (!section.interior) {
                append_cut(cuts, result.brep, face, std::move(cut), tolerance);
                continue;
            }
            // A curve strictly inside both faces lies along no edge and no other cut: it is taken as it is.
            place_cut(cut, section, result.node_vertices, corners);
            cut.family = second ? section.first_face : section.second_face;
            cuts.push_back(std::move(cut));
        }
        coincident_cuts(cuts, result.brep, face, other, sections, second, tolerance);
        join_cut_ends(cuts, result.brep, face, tolerance);
        if (!cuts.empty()) {
            const char* observation = std::getenv("SESSION_DODECA_PARTITION_OBSERVATION");
            if (observation && *observation && !second && face == 5
                && source.face_count() == 12 && other.face_count() == 12) {
                split::ArrangementDiagnostic diagnostic;
                observe_partition(observation, result.brep, cuts, diagnostic, tolerance, "input");
                try {
                    auto next = split::split_brep_face_by_pcurves(result.brep, face, cuts, tolerance, &diagnostic);
                    observe_partition(observation, result.brep, cuts, diagnostic, tolerance, "complete");
                    retain_lineage(result, std::move(next));
                } catch (...) {
                    observe_partition(observation, result.brep, cuts, diagnostic, tolerance, "exception");
                    throw;
                }
            } else if (const char* general = std::getenv("SESSION_BOOLEAN_PARTITION_OBSERVATION"); general && *general) {
                split::ArrangementDiagnostic diagnostic;
                try {
                    retain_lineage(result, split::split_brep_face_by_pcurves(result.brep, face, cuts, tolerance, &diagnostic));
                } catch (...) {
                    observe_partition(general, result.brep, cuts, diagnostic, tolerance, "exception", second ? 1 : 0, face);
                    throw;
                }
            } else {
                const split::FaceSplit next = split::split_brep_face_in_place(result.brep, face, cuts, tolerance, validated);
                retain_lineage(result, next.edges);
                validated = validated || next.faces.size() > 1;
            }
        }
    }
    mark_exact(result, source, sections, second);
    return result;
}

bool interior_candidate(
    const brep_classify::Solid& solid,
    int face,
    const Point& uv,
    double uv_tol,
    double tolerance,
    const brep_classify::Solid* avoid
) {
    if (brep_classify::face_uv(solid, face, uv, uv_tol) != State::Inside)
        return false;
    if (!avoid)
        return true;
    const NurbsSurface& surface = solid.brep.m_surfaces[solid.brep.m_faces[face].surface_index];
    const State state = brep_classify::point(*avoid, surface.point_at(uv[0], uv[1]), tolerance);
    if (state == State::Unknown)
        throw std::runtime_error("Native Boolean interior witness classification is unresolved");
    return state != State::Boundary;
}

Point interior(const brep_classify::Solid& solid, int face, double tolerance, const brep_classify::Solid* avoid = nullptr) {
    const BRep& brep = solid.brep;
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
            for (int i = 0; i < curve.cv_count(); ++i) {
                const xyz cv = curve.get_cv(i);
                for (int d = 0; d < 2; ++d) {
                    lo[d] = std::min(lo[d], cv[d]);
                    hi[d] = std::max(hi[d], cv[d]);
                }
            }
        }
    const double uv_tol = uv_tolerance(brep.m_surfaces[brep.m_faces[face].surface_index], tolerance);
    for (int n : {2, 4, 8, 16})
        for (int i = 1; i < n; ++i)
            for (int j = 1; j < n; ++j) {
                const Point uv(lo[0] + (hi[0] - lo[0]) * i / n, lo[1] + (hi[1] - lo[1]) * j / n, 0);
                if (interior_candidate(solid, face, uv, uv_tol, tolerance, avoid))
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
                if (interior_candidate(solid, face, uv, uv_tol, tolerance, avoid))
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
    const brep_classify::Solid& solid,
    const Point& uv,
    Point& other_uv,
    double tolerance
) {
    const BRep& other = solid.brep;
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
            || brep_classify::face_uv(solid, candidate, {parameters.first, parameters.second, 0},
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
    const brep_classify::Solid own(source), solid(other);
    const std::vector<int> faces = active_faces(source);
    // Faces joined across an edge the other solid's boundary does not meet lie on one side of it: one witness decides them all. Such an edge is an original one with no edge of the other operand along it or at its nodes; a pole joins nothing.
    std::vector<bool> open(source.edge_count(), false), seen(source.edge_count(), false);
    for (const split::EdgePiece& piece : partition.pieces)
        if (piece.edge >= 0 && piece.edge < source.edge_count() && !seen[piece.edge]) {
            seen[piece.edge] = true;
            open[piece.edge] = piece.section < 0 && piece.exact && !source.m_edges[piece.edge].degenerated;
        }
    std::vector<std::vector<int>> around(source.edge_count());
    for (int face : faces)
        for (const BRepRef& wire : source.m_faces[face].wires)
            for (const BRepRef& edge : source.wire_edges(wire))
                if (open[edge.index])
                    around[edge.index].push_back(face);
    std::vector<State> known(source.face_count(), State::Unknown);
    for (int face : faces) {
        State state = known[face];
        Point uv, other_uv;
        int coincident = -1;
        if (state == State::Unknown) {
            uv = interior(own, face, tolerance);
            const NurbsSurface& surface = source.m_surfaces[source.m_faces[face].surface_index];
            Point p = surface.point_at(uv[0], uv[1]);
            state = brep_classify::point(solid, p, tolerance);
            if (state == State::Unknown)
                throw ClassificationError(other, p, face, second, tolerance);
            coincident = state == State::Boundary ? coincident_region(source, face, solid, uv, other_uv, tolerance) : -1;
            if (state == State::Boundary && coincident < 0) {
                uv = interior(own, face, tolerance, &solid);
                p = surface.point_at(uv[0], uv[1]);
                state = brep_classify::point(solid, p, tolerance);
                if (state != State::Inside && state != State::Outside)
                    throw std::runtime_error("Native Boolean interior witness remained unresolved");
            }
            known[face] = state;
            std::vector<int> joined{face};
            for (size_t i = 0; state != State::Boundary && i < joined.size(); ++i)
                for (const BRepRef& wire : source.m_faces[joined[i]].wires)
                    for (const BRepRef& edge : source.wire_edges(wire))
                        for (int next : around[edge.index])
                            if (known[next] == State::Unknown) {
                                known[next] = state;
                                joined.push_back(next);
                            }
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

/// One curve of two that meet end to start, or false when they do not.
bool joined_curve(NurbsCurve a, NurbsCurve b, bool flip_a, bool flip_b, std::pair<double, double> domain_a,
                  std::pair<double, double> domain_b, double tolerance, NurbsCurve& out) {

    if (flip_a)
        a.reverse();
    if (flip_b)
        b.reverse();
    if (!a.set_domain(domain_a.first, domain_a.second) || !b.set_domain(domain_b.first, domain_b.second))
        return false;
    const std::vector<NurbsCurve> joined = NurbsCurve::join({a, b}, tolerance);
    if (joined.size() != 1 || joined[0].domain().first != domain_a.first ||
        joined[0].point_at_start().distance(a.point_at_start()) > tolerance ||
        joined[0].point_at_end().distance(b.point_at_end()) > tolerance)
        return false;
    out = joined[0];
    return true;
}

/// Keep each edge whole, as OCCT does: two edges at a vertex no other edge meets, used one after the other by the same wires
/// and lying on the same surfaces, become one. A section piece ends where it crosses a knot line of a NURBS surface or where
/// a closed section closes; an analytic surface has neither.
BRep joined_edges(BRep b, double tolerance) {

    const BRepOrientation F = BRepOrientation::Forward;
    const BRepOrientation R = BRepOrientation::Reversed;
    std::vector<bool> dropped_edge(b.edge_count(), false), dropped_vertex(b.vertex_count(), false);
    std::vector<bool> stale_3d(b.m_curves_3d.size(), false), stale_2d(b.m_curves_2d.size(), false);

    for (int v = 0; v < b.vertex_count(); ++v) {
        std::vector<std::pair<int, bool>> ends;
        for (int e = 0; e < b.edge_count(); ++e)
            for (bool last : {false, true})
                if (!dropped_edge[e] && (last ? b.m_edges[e].end_vertex : b.m_edges[e].start_vertex) == v)
                    ends.push_back({e, last});
        if (ends.size() != 2 || ends[0].first == ends[1].first)
            continue;
        const int e1 = ends[0].first, e2 = ends[1].first;
        const bool flip1 = !ends[0].second, flip2 = ends[1].second;
        BRepEdge& x = b.m_edges[e1];
        const BRepEdge& y = b.m_edges[e2];
        if (x.degenerated || y.degenerated || x.pcurves.size() != y.pcurves.size())
            continue;

        std::vector<std::array<int, 3>> uses;
        bool consecutive = true;
        for (int w = 0; w < (int)b.m_wires.size() && consecutive; ++w) {
            const std::vector<BRepRef>& refs = b.m_wires[w].edges;
            const int n = (int)refs.size();
            int i1 = -1, i2 = -1, count1 = 0, count2 = 0;
            for (int i = 0; i < n; ++i)
                if (refs[i].index == e1) {
                    i1 = i;
                    ++count1;
                } else if (refs[i].index == e2) {
                    i2 = i;
                    ++count2;
                }
            if (count1 == 0 && count2 == 0)
                continue;
            consecutive = count1 == 1 && count2 == 1 && (refs[i1].orientation == F || refs[i1].orientation == R) &&
                (refs[i2].orientation == F || refs[i2].orientation == R);
            const bool toward = consecutive && (refs[i1].orientation == F) != flip1;
            consecutive = consecutive && i2 == (toward ? (i1 + 1) % n : (i1 + n - 1) % n) &&
                ((refs[i2].orientation == F) != flip2) == toward;
            uses.push_back({w, i1, i2});
        }
        if (!consecutive || uses.empty())
            continue;

        const std::pair<double, double> d1 = b.m_curves_3d[x.curve_3d_index].domain();
        const std::pair<double, double> d2 = b.m_curves_3d[y.curve_3d_index].domain();
        NurbsCurve curve;
        if (!joined_curve(b.m_curves_3d[x.curve_3d_index], b.m_curves_3d[y.curve_3d_index], flip1, flip2, d1, d2, tolerance, curve))
            continue;
        std::vector<BRepCurveOnSurface> pcurves;
        std::vector<NurbsCurve> curves_2d;
        for (const BRepCurveOnSurface& p : x.pcurves) {
            const auto q = std::find_if(y.pcurves.begin(), y.pcurves.end(), [&](const BRepCurveOnSurface& c) { return c.surface_index == p.surface_index; });
            if (q == y.pcurves.end() || (p.curve_2d_index_2 < 0) != (q->curve_2d_index_2 < 0))
                break;
            BRepCurveOnSurface joined{p.surface_index, -1, -1};
            for (bool second : {false, true}) {
                if (second && p.curve_2d_index_2 < 0)
                    continue;
                const int a = second != flip1 && p.curve_2d_index_2 >= 0 ? p.curve_2d_index_2 : p.curve_2d_index;
                const int c = second != flip2 && q->curve_2d_index_2 >= 0 ? q->curve_2d_index_2 : q->curve_2d_index;
                NurbsCurve pcurve;
                if (!joined_curve(b.m_curves_2d[a], b.m_curves_2d[c], flip1, flip2, d1, d2, tolerance, pcurve))
                    break;
                (second ? joined.curve_2d_index_2 : joined.curve_2d_index) = (int)(b.m_curves_2d.size() + curves_2d.size());
                curves_2d.push_back(pcurve);
            }
            if (joined.curve_2d_index < 0 || (p.curve_2d_index_2 >= 0 && joined.curve_2d_index_2 < 0))
                break;
            pcurves.push_back(joined);
        }
        if (pcurves.size() != x.pcurves.size())
            continue;

        for (int e : {e1, e2}) {
            stale_3d[b.m_edges[e].curve_3d_index] = true;
            for (const BRepCurveOnSurface& p : b.m_edges[e].pcurves)
                for (int index : {p.curve_2d_index, p.curve_2d_index_2})
                    if (index >= 0)
                        stale_2d[index] = true;
        }
        for (const std::array<int, 3>& use : uses) {
            std::vector<BRepRef>& refs = b.m_wires[use[0]].edges;
            const bool toward = (refs[use[1]].orientation == F) != flip1;
            refs[use[1]] = {e1, toward ? F : R};
            refs.erase(refs.begin() + use[2]);
        }
        x.start_vertex = flip1 ? x.end_vertex : x.start_vertex;
        x.end_vertex = flip2 ? y.start_vertex : y.end_vertex;
        x.tolerance = std::max(x.tolerance, y.tolerance);
        x.curve_3d_index = b.add_curve_3d(curve);
        x.pcurves = pcurves;
        b.m_curves_2d.insert(b.m_curves_2d.end(), curves_2d.begin(), curves_2d.end());
        stale_3d.push_back(false);
        stale_2d.resize(b.m_curves_2d.size(), false);
        dropped_edge[e2] = true;
        dropped_vertex[v] = true;
    }

    const auto remap = [](auto& items, const std::vector<bool>& drop) {
        std::vector<int> index(items.size(), -1);
        std::remove_reference_t<decltype(items)> kept;
        for (size_t i = 0; i < items.size(); ++i)
            if (!drop[i]) {
                index[i] = (int)kept.size();
                kept.push_back(std::move(items[i]));
            }
        items = std::move(kept);
        return index;
    };
    for (int e = 0; e < b.edge_count(); ++e) {
        if (dropped_edge[e])
            continue;
        if (b.m_edges[e].curve_3d_index >= 0)
            stale_3d[b.m_edges[e].curve_3d_index] = false;
        for (const BRepCurveOnSurface& p : b.m_edges[e].pcurves)
            for (int index : {p.curve_2d_index, p.curve_2d_index_2})
                if (index >= 0)
                    stale_2d[index] = false;
    }
    const std::vector<int> curve_3d = remap(b.m_curves_3d, stale_3d);
    const std::vector<int> curve_2d = remap(b.m_curves_2d, stale_2d);
    const std::vector<int> vertex = remap(b.m_vertices, dropped_vertex);
    const std::vector<int> edge = remap(b.m_edges, dropped_edge);
    for (BRepEdge& e : b.m_edges) {
        e.start_vertex = vertex[e.start_vertex];
        e.end_vertex = vertex[e.end_vertex];
        if (e.curve_3d_index >= 0)
            e.curve_3d_index = curve_3d[e.curve_3d_index];
        for (BRepCurveOnSurface& p : e.pcurves)
            for (int* index : {&p.curve_2d_index, &p.curve_2d_index_2})
                if (*index >= 0)
                    *index = curve_2d[*index];
    }
    for (BRepWire& w : b.m_wires)
        for (BRepRef& r : w.edges)
            r.index = edge[r.index];

    return b;
}

bool empty(const BRep& brep) {
    return brep.m_faces.empty() && brep.m_vertices.empty() && brep.m_edges.empty()
        && brep.m_wires.empty() && brep.m_shells.empty() && brep.m_solids.empty();
}

void observe_assembly_failure(const char* path, const BRep& first, const BRep& second,
    const Partition& a, const Partition& b, const std::vector<brep_assemble::FaceUse>& selected,
    Operation operation, double tolerance, const brep_assemble::TopologyError& error) {
    nlohmann::ordered_json row = {{"schema", "native_boolean_assembly_failure_v1"},
        {"failure", error.what()}, {"operation", static_cast<int>(operation)}, {"tolerance", tolerance},
        {"operands", {first.jsondump(), second.jsondump()}}, {"assembly_shapes", nlohmann::ordered_json::array()},
        {"assembly_faces", nlohmann::ordered_json::array()}, {"partial", error.partial.jsondump()},
        {"face_orientations", nlohmann::ordered_json::array()}, {"images", nlohmann::ordered_json::array()},
        {"input_index_contract", "image.input is the first selected FaceUse for its BRep owner; assembly_faces[input].shape identifies its unique owner"}};
    row["assembly_stage"] = error.stage;
    row["orientation_attempts"] = nlohmann::ordered_json::array();
    const auto xyz = [](const Point& p) { return std::array<double, 3>{p[0], p[1], p[2]}; };
    row["containment_attempts"] = nlohmann::ordered_json::array();
    for (const auto& attempt : error.containment_attempts)
        row["containment_attempts"].push_back({{"shell", attempt.shell},
            {"reference_shell", attempt.reference_shell}, {"face", attempt.face},
            {"uv", xyz(attempt.uv)}, {"point", xyz(attempt.point)},
            {"state", static_cast<int>(attempt.state)}});
    for (const auto& attempt : error.orientation_attempts) {
        nlohmann::ordered_json hits = nlohmann::ordered_json::array();
        for (const auto& hit : attempt.ray.hits)
            hits.push_back({{"point", xyz(hit.point)}, {"distance", hit.distance},
                {"distance_enclosure", hit.distance_enclosure}, {"face", hit.face},
                {"uv", {hit.u, hit.v}}, {"trim", static_cast<int>(hit.trim)}, {"crossing", hit.crossing}});
        row["orientation_attempts"].push_back({{"shell", attempt.shell}, {"face", attempt.face},
            {"axis", attempt.axis}, {"side", attempt.side}, {"uv", xyz(attempt.uv)},
            {"origin", xyz(attempt.origin)}, {"target", xyz(attempt.target)},
            {"complete", attempt.ray.complete}, {"failure", attempt.ray.failure},
            {"unresolved_face", attempt.ray.unresolved_face},
            {"unresolved_domain", attempt.ray.unresolved_domain}, {"hits", std::move(hits)},
            {"crossing", attempt.crossing}, {"decision", attempt.reason}});
    }
    for (const Partition* partition : {&a, &b}) {
        nlohmann::ordered_json pieces = nlohmann::ordered_json::array();
        for (const auto& piece : partition->pieces)
            pieces.push_back({{"source_edge", piece.source_edge}, {"section", piece.section}, {"edge", piece.edge},
                {"first", piece.first}, {"last", piece.last}});
        row["assembly_shapes"].push_back({{"brep", partition->brep.jsondump()},
            {"operand", partition->operand}, {"pieces", std::move(pieces)}});
    }
    for (const auto& face : selected) {
        const int owner = face.brep == &a.brep ? 0 : face.brep == &b.brep ? 1 : -1;
        if (owner < 0)
            throw std::runtime_error("Assembly failure has an unknown selected shape owner");
        row["assembly_faces"].push_back({{"shape", owner}, {"face", face.face},
            {"reversed", face.reversed}, {"operand", face.operand}});
    }
    for (const auto orientation : error.face_orientations)
        row["face_orientations"].push_back(static_cast<int>(orientation));
    for (const auto& image : error.images)
        row["images"].push_back({{"input", image.input}, {"local_edge", image.local_edge},
            {"original_edge", image.original_edge}, {"section", image.section}, {"output_edge", image.output_edge},
            {"first", image.first}, {"last", image.last}, {"reversed", image.reversed}});
    std::ofstream output(path, std::ios::app | std::ios::binary);
    if (!output || !(output << row.dump() << '\n'))
        throw std::runtime_error(std::string("Cannot write Boolean assembly failure observation: ") + path);
    output.flush();
    if (!output)
        throw std::runtime_error(std::string("Cannot flush Boolean assembly failure observation: ") + path);
    output.close();
    if (!output)
        throw std::runtime_error(std::string("Cannot close Boolean assembly failure observation: ") + path);
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
    nurbs_memo::Scope reuse;
    boolean_progress("section");
    const brep_section::FaceResult sections = brep_section::interferences(first, second, tolerance);
    if (!sections.complete)
        throw std::runtime_error(sections.failure);
    if (std::getenv("SESSION_BOOLEAN_PROGRESS")) {
        std::fprintf(stderr,"boolean_interferences execution_complete=1 coverage_complete=%d boundary_ownership_complete=%d pairs=%zu boundary_events=%zu nodes=%zu uv_only=%zu unrepresented_uv=%zu\n",
            sections.coverage_complete()?1:0,sections.boundary_ownership_complete?1:0,sections.coverage.size(),
            sections.boundary_events.size(),sections.nodes.size(),sections.uv_only_incidences.size(),
            sections.unrepresented_uv_incidences.size());
        std::fflush(stderr);
    }
    boolean_progress("partition_a");
    const Partition a = partition(first, second, sections, false, tolerance);
    boolean_progress("partition_b");
    const Partition b = partition(second, first, sections, true, tolerance);
    std::vector<brep_assemble::FaceUse> selected;
    boolean_progress("select_a");
    select(selected, a, b, false, operation, tolerance);
    boolean_progress("select_b");
    select(selected, b, a, true, operation, tolerance);
    boolean_progress("assemble");
    try {
        BRep result = joined_edges(brep_assemble::assemble(selected, tolerance), tolerance);
        boolean_progress("complete");
        return result;
    } catch (const brep_assemble::TopologyError& error) {
        const char* observation = std::getenv("SESSION_BOOLEAN_FAILURE_OBSERVATION");
        if (observation && *observation)
            try {
                observe_assembly_failure(observation, first, second, a, b, selected, operation, tolerance, error);
            } catch (const std::exception& capture_error) {
                std::fprintf(stderr, "Boolean failure observation failed: %s\n", capture_error.what());
                std::fflush(stderr);
            } catch (...) {
                std::fprintf(stderr, "Boolean failure observation failed with unknown error\n");
                std::fflush(stderr);
            }
        throw;
    }
}

}
