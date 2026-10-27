#include "brep_offset_inward.h"
#include "brep_assemble.h"
#include "brep_section.h"
#include "curve_intersection.h"
#include "curve_surface_intersection.h"
#include "split.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>

namespace session_cpp::brep_offset {
namespace {

constexpr BRepOrientation Forward = BRepOrientation::Forward;
constexpr BRepOrientation Reversed = BRepOrientation::Reversed;
constexpr double Epsilon = std::numeric_limits<double>::epsilon();

void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

Vector unit(const Vector& vector) {
    const double length = std::hypot(vector[0], vector[1], vector[2]);
    require(std::isfinite(length) && length > 0.0, "Undefined inward differential direction");
    return vector / length;
}

Vector normal(const NurbsSurface& surface, const Point& uv) {
    const std::vector<Vector> derivatives = surface.evaluate(uv[0], uv[1], 1);
    const Vector area = unit(derivatives[2]).cross(unit(derivatives[1]));
    require(area.magnitude_squared() > 16384.0 * Epsilon * Epsilon, "Singular inward support normal");
    return unit(area);
}

struct Use {
    int face;
    BRepOrientation orientation;
};

struct Section {
    bool valid = false;
    NurbsCurve world;
    std::array<NurbsCurve, 2> pcurves;
    std::array<std::vector<double>, 2> events;
    std::array<double, 2> cuts{};
};

struct Context {
    const BRep& source;
    std::vector<OffsetFace>& faces;
    double distance;
    double tolerance;
    std::string stage = "input topology";
    int edge = -1;
    int vertex = -1;
    int active_face = -1;
    std::vector<std::vector<Use>> uses;
    std::vector<std::vector<int>> vertices;
    std::vector<std::vector<int>> face_edges;
    std::vector<Section> sections;
    std::vector<Point> points;

    const NurbsSurface& surface(int face) const {
        const BRep& brep = faces[face].brep;
        return brep.m_surfaces[brep.m_faces[0].surface_index];
    }

    int side(int edge_index, int face) const {
        for (int i = 0; i < 2; ++i)
            if (uses[edge_index][i].face == face)
                return i;
        throw std::runtime_error("Section lacks its expected incident face");
    }

    const NurbsCurve& pcurve(int edge_index, int face) const {
        return sections[edge_index].pcurves[side(edge_index, face)];
    }
};

InwardError diagnostic(const Context& context, const std::exception& error);

void collect_topology(Context& context) {
    const BRep& source = context.source;
    require(source.is_valid() && source.is_solid() && source.solid_count() == 1
        && source.m_solids[0].shells.size() == 1, "Inward joins require one closed source shell");
    const BRepShell& shell = source.m_shells[source.m_solids[0].shells[0].index];
    require(context.faces.size() == shell.faces.size(), "Inward joins require every source face");
    context.uses.resize(source.edge_count());
    context.vertices.resize(source.vertex_count());
    context.face_edges.resize(context.faces.size());
    context.sections.resize(source.edge_count());
    context.points.resize(source.vertex_count());
    std::set<int> visited;
    for (int i = 0; i < static_cast<int>(context.faces.size()); ++i) {
        const OffsetFace& displaced = context.faces[i];
        require(displaced.source >= 0 && displaced.source < source.face_count()
            && visited.insert(displaced.source).second, "Repeated or invalid inward source face");
        require(std::any_of(shell.faces.begin(), shell.faces.end(), [&](const BRepRef& face) {
            return face.index == displaced.source;
        }), "Displaced face is outside the selected source shell");
        require(displaced.brep.is_valid() && displaced.brep.face_count() == 1,
            "Inward construction requires one valid displaced face per source face");
        const BRepFace& face = source.m_faces[displaced.source];
        require(face.wires.size() == 1, "Inward construction requires one outer wire per source face");
        const BRepOrientation orientation = source.face_orientation(displaced.source);
        require(orientation == Forward || orientation == Reversed, "Unsupported inward face orientation");
        for (const BRepRef& edge : source.wire_edges(face.wires[0])) {
            require(edge.orientation == Forward || edge.orientation == Reversed, "Unsupported inward edge orientation");
            context.uses[edge.index].push_back({i, edge.orientation});
            context.face_edges[i].push_back(edge.index);
        }
    }
}

void validate_incidence(Context& context) {
    std::set<std::pair<int, int>> pairs;
    for (int edge = 0; edge < context.source.edge_count(); ++edge) {
        const std::vector<Use>& uses = context.uses[edge];
        if (uses.empty())
            continue;
        const BRepEdge& source = context.source.m_edges[edge];
        require(uses.size() == 2 && uses[0].face != uses[1].face && !source.degenerated
            && source.start_vertex != source.end_vertex, "Inward joins require distinct two-face edges without seams or poles");
        require(pairs.insert(std::minmax(uses[0].face, uses[1].face)).second,
            "Inward joins currently require one source edge per adjacent face pair");
        for (int vertex : {source.start_vertex, source.end_vertex})
            context.vertices[vertex].push_back(edge);
        const BRepOrientation first = brep_compose(context.source.face_orientation(context.faces[uses[0].face].source), uses[0].orientation);
        const BRepOrientation second = brep_compose(context.source.face_orientation(context.faces[uses[1].face].source), uses[1].orientation);
        require(first != second, "Inward source edge uses have inconsistent orientation");
    }
    for (const std::vector<int>& edges : context.vertices)
        require(edges.empty() || edges.size() == 3, "Inward joins currently require valence-three vertices");
    std::vector<bool> reached(context.faces.size(), false);
    reached[0] = true;
    for (size_t pass = 0; pass < context.faces.size(); ++pass)
        for (const std::vector<Use>& edge : context.uses)
            if (!edge.empty() && (reached[edge[0].face] || reached[edge[1].face]))
                reached[edge[0].face] = reached[edge[1].face] = true;
    require(std::all_of(reached.begin(), reached.end(), [](bool value) { return value; }),
        "Inward joins require a connected shell");
}

Vector source_normal(const Context& context, const Use& use, double parameter) {
    const int face = context.faces[use.face].source;
    const BRep& source = context.source;
    const int index = source.pcurve_index(context.edge, face, use.orientation);
    require(index >= 0, "Inward source edge lacks an incident pcurve");
    const NurbsCurve& pcurve = source.m_curves_2d[index];
    const double t = pcurve.domain_start() + parameter * (pcurve.domain_end() - pcurve.domain_start());
    const Vector direction = normal(source.m_surfaces[source.m_faces[face].surface_index], pcurve.point_at(t));
    return source.face_orientation(face) == Reversed ? -direction : direction;
}

void check_convex(const Context& context) {
    const std::vector<Use>& uses = context.uses[context.edge];
    const BRepEdge& edge = context.source.m_edges[context.edge];
    const NurbsCurve& curve = context.source.m_curves_3d[edge.curve_3d_index];
    const BRepOrientation orientation = brep_compose(context.source.face_orientation(context.faces[uses[0].face].source), uses[0].orientation);
    const double sign = orientation == Forward ? 1.0 : -1.0;
    const double angular = std::min(0.01, context.tolerance / std::abs(context.distance));
    for (int i = 1; i < 48; ++i) {
        const double t = i / 48.0;
        const double parameter = curve.domain_start() + t * (curve.domain_end() - curve.domain_start());
        const Vector tangent = unit(curve.evaluate(parameter, 1)[1]);
        const Vector first = source_normal(context, uses[0], t);
        const Vector second = source_normal(context, uses[1], t);
        require(sign * tangent.dot(first.cross(second)) > angular,
            "Inward construction requires consistently convex sharp joins; smooth, concave or mixed joins need another path");
    }
}

void certify(const NurbsCurve& world, const NurbsCurve& pcurve, const NurbsSurface& surface, double tolerance) {
    const curve_surface_intersection::ParameterMatch match = curve_surface_intersection::same_parameter(world, pcurve, surface, tolerance);
    if (match != curve_surface_intersection::ParameterMatch::Matched)
        throw brep_assemble::ParameterError("Inward section lacks certified paired parameters", world, pcurve, surface, tolerance, match);
}

void certify_source_edge(const Context& context) {
    const BRep& source = context.source;
    const NurbsCurve& world = source.m_curves_3d[source.m_edges[context.edge].curve_3d_index];
    for (const Use& use : context.uses[context.edge]) {
        const int face = context.faces[use.face].source;
        const int index = source.pcurve_index(context.edge, face, use.orientation);
        require(index >= 0, "Inward source lacks an incident pcurve");
        certify(world, source.m_curves_2d[index], source.m_surfaces[source.m_faces[face].surface_index], context.tolerance * 0.125);
    }
}

void intersect_neighbors(Context& context) {
    context.stage = "adjacent displaced supports";
    for (int edge = 0; edge < context.source.edge_count(); ++edge) {
        if (context.uses[edge].empty())
            continue;
        context.edge = edge;
        certify_source_edge(context);
        check_convex(context);
        const int first = context.uses[edge][0].face;
        const int second = context.uses[edge][1].face;
        const brep_section::FaceResult result = brep_section::faces(context.faces[first].brep,
            context.faces[second].brep, context.tolerance * 0.0625);
        require(result.complete, result.failure);
        require(result.overlaps.empty() && result.curves.size() == 1,
            "Inward adjacent faces require exactly one transverse section branch");
        Section& section = context.sections[edge];
        section.valid = true;
        section.world = result.curves[0].world;
        section.pcurves = {result.curves[0].first_pcurve, result.curves[0].second_pcurve};
        require(section.world.set_domain(0, 1), "Could not normalize inward section range");
        for (int side = 0; side < 2; ++side) {
            require(section.pcurves[side].set_domain(0, 1), "Could not normalize inward pcurve range");
            certify(section.world, section.pcurves[side], context.surface(context.uses[edge][side].face), context.tolerance * 0.25);
        }
    }
    context.edge = -1;
}

NurbsCurve normalized_uv(const NurbsCurve& curve, const NurbsSurface& surface) {
    NurbsCurve result = curve;
    for (int i = 0; i < result.cv_count(); ++i)
        for (int direction = 0; direction < 2; ++direction) {
            const std::pair<double, double> domain = surface.domain(direction);
            result.cv(i)[direction] = (curve.cv(i)[direction] - domain.first * curve.weight(i))
                / (domain.second - domain.first);
        }
    return result;
}

double uv_tolerance(const NurbsSurface& surface, double tolerance) {
    Point low = surface.get_cv(0, 0);
    Point high = low;
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j)
            for (int d = 0; d < 3; ++d) {
                low[d] = std::min(low[d], surface.get_cv(i, j)[d]);
                high[d] = std::max(high[d], surface.get_cv(i, j)[d]);
            }
    const double diameter = low.distance(high);
    require(std::isfinite(diameter) && diameter > 0.0, "Inward support has a collapsed control hull");
    const double result = tolerance * 0.0625 / diameter;
    require(result > 64.0 * Epsilon, "Inward crossings are below parameter resolution");
    return result;
}

int common_face(const Context& context, int first, int second) {
    int result = -1;
    for (const Use& a : context.uses[first])
        for (const Use& b : context.uses[second])
            if (a.face == b.face) {
                require(result < 0, "Ambiguous inward vertex face incidence");
                result = a.face;
            }
    require(result >= 0, "Inward vertex sections do not share a face");
    return result;
}

void vertex_event(Context& context, int first, int second) {
    const int face = common_face(context, first, second);
    const NurbsSurface& surface = context.surface(face);
    const curve_intersection::Result result = curve_intersection::intersect(
        normalized_uv(context.pcurve(first, face), surface),
        normalized_uv(context.pcurve(second, face), surface), uv_tolerance(surface, context.tolerance));
    require(result.complete, result.failure);
    require(result.overlaps.empty() && result.points.size() == 1 && !result.points[0].tangent,
        "Inward triple vertex requires one transverse pcurve crossing per incident face");
    const curve_intersection::Hit& hit = result.points[0];
    for (const std::pair<int, double>& cut : {std::pair<int, double>{first, hit.a}, {second, hit.b}}) {
        const BRepEdge& edge = context.source.m_edges[cut.first];
        const int end = edge.start_vertex == context.vertex ? 0 : 1;
        require(edge.start_vertex == context.vertex || edge.end_vertex == context.vertex,
            "Inward crossing lost source vertex ancestry");
        context.sections[cut.first].events[end].push_back(cut.second);
    }
}

void reconcile_vertex(Context& context) {
    const std::vector<int>& edges = context.vertices[context.vertex];
    std::vector<Point> positions;
    std::map<int, Vector> normals;
    for (int edge : edges) {
        Section& section = context.sections[edge];
        const int end = context.source.m_edges[edge].start_vertex == context.vertex ? 0 : 1;
        const std::vector<double>& events = section.events[end];
        require(events.size() == 2, "Inward vertex lacks its two incident parameter observations");
        const double parameter = (events[0] + events[1]) * 0.5;
        const Point point = section.world.point_at(parameter);
        for (double event : events)
            require(point.distance(section.world.point_at(event)) <= context.tolerance * 0.125,
                "Incident inward pcurve crossings disagree in world space");
        section.cuts[end] = parameter;
        positions.push_back(point);
        for (int side = 0; side < 2; ++side) {
            const int face = context.uses[edge][side].face;
            const Point uv = section.pcurves[side].point_at(parameter);
            positions.push_back(context.surface(face).point_at(uv[0], uv[1]));
            normals[face] = normal(context.surface(face), uv);
        }
    }
    Vector shift;
    for (const Point& point : positions)
        shift += point - positions[0];
    const Point center = positions[0] + shift / static_cast<double>(positions.size());
    for (const Point& point : positions)
        require(point.distance(center) <= context.tolerance * 0.25,
            "Inward sections do not define one shared triple vertex");
    require(normals.size() == 3, "Inward vertex must have three distinct incident supports");
    auto it = normals.begin();
    const Vector a = (it++)->second;
    const Vector b = (it++)->second;
    const Vector c = it->second;
    require(std::abs(a.dot(b.cross(c))) > 512.0 * Epsilon,
        "Inward triple supports are singular or tangent");
    context.points[context.vertex] = center;
}

void synchronize_vertices(Context& context) {
    context.stage = "triple vertex pcurve intersections";
    for (int vertex = 0; vertex < context.source.vertex_count(); ++vertex) {
        const std::vector<int>& edges = context.vertices[vertex];
        if (edges.empty())
            continue;
        context.vertex = vertex;
        for (int i = 0; i < 3; ++i)
            for (int j = i + 1; j < 3; ++j)
                vertex_event(context, edges[i], edges[j]);
    }
    context.stage = "shared triple vertex reconciliation";
    for (int vertex = 0; vertex < context.source.vertex_count(); ++vertex)
        if (!context.vertices[vertex].empty()) {
            context.vertex = vertex;
            reconcile_vertex(context);
        }
    context.vertex = -1;
}

NurbsCurve trimmed(const NurbsCurve& curve, const std::array<double, 2>& ends) {
    NurbsCurve result = curve;
    require(std::abs(ends[1] - ends[0]) > 128.0 * Epsilon, "Inward section collapses between triple vertices");
    require(result.trim(std::min(ends[0], ends[1]), std::max(ends[0], ends[1])), "Could not trim inward section");
    if (ends[1] < ends[0])
        require(result.reverse(), "Could not orient inward section from source edge ancestry");
    require(result.set_domain(0, 1), "Could not synchronize trimmed inward section parameters");
    return result;
}

void trim_sections(Context& context) {
    context.stage = "synchronized section trimming";
    for (int edge = 0; edge < context.source.edge_count(); ++edge) {
        Section& section = context.sections[edge];
        if (!section.valid)
            continue;
        context.edge = edge;
        section.world = trimmed(section.world, section.cuts);
        for (int side = 0; side < 2; ++side) {
            section.pcurves[side] = trimmed(section.pcurves[side], section.cuts);
            certify(section.world, section.pcurves[side], context.surface(context.uses[edge][side].face), context.tolerance * 0.25);
        }
        const BRepEdge& source = context.source.m_edges[edge];
        require(section.world.point_at_start().distance(context.points[source.start_vertex]) <= context.tolerance * 0.25
            && section.world.point_at_end().distance(context.points[source.end_vertex]) <= context.tolerance * 0.25,
            "Trimmed inward section lost its shared triple vertices");
    }
    context.edge = -1;
}

bool complete_sections(
    const split::FaceSplit& result,
    const BRepRef& wire,
    const std::map<int, const split::EdgePiece*>& ancestry,
    const std::vector<int>& expected
) {
    std::map<int, std::vector<std::pair<double, double>>> intervals;
    std::set<int> edges;
    for (const BRepRef& edge : result.brep.wire_edges(wire)) {
        const auto found = ancestry.find(edge.index);
        if (found == ancestry.end() || found->second->section < 0 || !edges.insert(edge.index).second)
            return false;
        const split::EdgePiece& piece = *found->second;
        const double first = std::min(piece.first, piece.last);
        const double last = std::max(piece.first, piece.last);
        if (!std::isfinite(first) || !std::isfinite(last) || first >= last)
            return false;
        intervals[piece.section].push_back({first, last});
    }
    if (intervals.size() != expected.size())
        return false;
    // MakeLoops reuses every cut image of a section. Here its shared range is [0,1].
    for (int section : expected) {
        const auto found = intervals.find(section);
        if (found == intervals.end())
            return false;
        auto& pieces = found->second;
        std::sort(pieces.begin(), pieces.end());
        double end = 0.0;
        for (const auto& piece : pieces) {
            if (std::abs(piece.first - end) > 128.0 * Epsilon)
                return false;
            end = piece.second;
        }
        if (std::abs(end - 1.0) > 128.0 * Epsilon)
            return false;
    }
    return true;
}

int retained_face(const split::FaceSplit& result, const std::vector<int>& expected) {
    std::map<int, const split::EdgePiece*> ancestry;
    for (const split::EdgePiece& piece : result.edges)
        ancestry[piece.edge] = &piece;
    int selected = -1;
    for (int face : result.faces) {
        const BRepFace& candidate = result.brep.m_faces[face];
        if (candidate.wires.size() != 1)
            continue;
        if (!complete_sections(result, candidate.wires[0], ancestry, expected))
            continue;
        require(selected < 0, "Inward arrangement has multiple retained section-only regions");
        selected = face;
    }
    require(selected >= 0, "Inward arrangement has no single section-only region with the original edge ancestry");
    return selected;
}

BRep rebuild_faces(Context& context) {
    context.stage = "face arrangements";
    std::vector<split::FaceSplit> partitions;
    std::vector<int> retained;
    for (int face = 0; face < static_cast<int>(context.faces.size()); ++face) {
        context.active_face = face;
        std::vector<split::FaceCut> cuts;
        for (int edge : context.face_edges[face])
            cuts.push_back({context.sections[edge].world, context.pcurve(edge, face), edge});
        partitions.push_back(split::split_brep_face_by_pcurves(context.faces[face].brep, 0, cuts, context.tolerance * 0.5));
        try {
            retained.push_back(retained_face(partitions.back(), context.face_edges[face]));
        } catch (const std::exception& failure) {
            InwardError error = diagnostic(context, failure);
            error.partition = partitions.back().brep;
            error.candidate_faces = partitions.back().faces;
            error.pieces = partitions.back().edges;
            throw error;
        }
    }
    context.active_face = -1;
    context.stage = "closed shell assembly";
    std::vector<brep_assemble::FaceUse> selected;
    for (size_t i = 0; i < partitions.size(); ++i)
        selected.push_back({&partitions[i].brep, retained[i], false, &partitions[i].edges, -1});
    BRep result = brep_assemble::assemble(selected, context.tolerance);
    require(result.is_valid() && result.is_solid() && result.solid_count() == 1 && result.shell_count() == 1,
        "Inward joins did not reconstruct one closed oriented solid");
    require(result.face_count() == static_cast<int>(context.faces.size()), "Inward face ancestry changed unexpectedly");
    return result;
}

InwardError diagnostic(const Context& context, const std::exception& error) {
    InwardError result(std::string(error.what()) + "; inward stage " + context.stage
        + "; source edge " + std::to_string(context.edge) + "; source vertex " + std::to_string(context.vertex));
    result.stage = context.stage;
    result.source_edge = context.edge;
    result.source_vertex = context.vertex;
    result.source_face = context.active_face >= 0 ? context.faces[context.active_face].source : -1;
    for (size_t i = 0; i < context.faces.size(); ++i)
        if (context.faces[i].brep.face_count() == 1 && context.faces[i].brep.is_valid()) {
            result.source_faces.push_back(context.faces[i].source);
            result.surfaces.push_back(context.surface(static_cast<int>(i)));
        }
    for (size_t edge = 0; edge < context.sections.size(); ++edge)
        if (context.sections[edge].valid) {
            result.section_edges.push_back(static_cast<int>(edge));
            result.curves.push_back(context.sections[edge].world);
            result.first_pcurves.push_back(context.sections[edge].pcurves[0]);
            result.second_pcurves.push_back(context.sections[edge].pcurves[1]);
        }
    return result;
}

}

BRep inward(const BRep& source, std::vector<OffsetFace> faces, double distance, double tolerance) {
    if (!std::isfinite(distance) || distance >= 0.0 || !std::isfinite(tolerance) || tolerance <= 0.0)
        throw std::invalid_argument("Inward joins require a finite negative distance and positive tolerance");
    Context context{source, faces, distance, tolerance};
    try {
        collect_topology(context);
        validate_incidence(context);
        intersect_neighbors(context);
        synchronize_vertices(context);
        trim_sections(context);
        return rebuild_faces(context);
    } catch (const InwardError&) {
        throw;
    } catch (const brep_assemble::ParameterError&) {
        throw;
    } catch (const std::exception& error) {
        throw diagnostic(context, error);
    }
}

}
