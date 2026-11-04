#include "brep_hollow.h"
#include "brep_hollow_affine.h"
#include "brep_hollow_source_extension.h"
#include "brep_offset_inward.h"
#include "brep_offset.h"
#include "brep_assemble.h"
#include "brep_classify.h"
#include "curve_intersection.h"
#include "nurbs_coincidence.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <set>
#include <stdexcept>

namespace session_cpp::brep_hollow {
namespace {
void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

std::set<int> validate(const BRep& source, const std::vector<int>& openings, double thickness, double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0 || !std::isfinite(thickness) || thickness == 0
        || !source.is_valid() || !source.is_solid())
        throw std::invalid_argument("Hollowing requires a valid solid, finite nonzero thickness and positive tolerance");
    std::set<int> removed;
    for (int face : openings) {
        if (face < 0 || face >= source.face_count())
            throw std::out_of_range("Hollowing face index is out of range");
        removed.insert(face);
    }
    if (removed.empty())
        return removed;
    require(thickness < 0, "Native hollowing outward arc joins at openings are not implemented");
    require(source.solid_count() == 1 && source.m_solids[0].shells.size() == 1,
        "Native hollowing currently requires one source shell");
    const BRepShell& shell = source.m_shells[source.m_solids[0].shells[0].index];
    std::set<int> active;
    for (const BRepRef& face : shell.faces)
        active.insert(face.index);
    require(active.size() == source.m_faces.size() && removed.size() < active.size(),
        "Native hollowing requires active source faces and at least one retained wall");
    std::map<int, int> mouths;
    for (int face : removed) {
        require(source.m_faces[face].wires.size() == 1, "Native hollow openings currently require one outer wire");
        for (const BRepRef& edge : source.wire_edges(source.m_faces[face].wires[0]))
            require(++mouths[edge.index] == 1, "Native hollowing adjacent openings require a combined rim arrangement");
    }
    return removed;
}

std::vector<split::EdgePiece> identities(const BRep& brep, int base) {
    std::vector<split::EdgePiece> result;
    for (int edge = 0; edge < brep.edge_count(); ++edge) {
        const NurbsCurve& curve = brep.m_curves_3d[brep.m_edges[edge].curve_3d_index];
        result.push_back({edge, base + edge, edge, curve.domain_start(), curve.domain_end()});
    }
    return result;
}

void certify(const NurbsCurve& world, const NurbsCurve& pc, const NurbsSurface& surface, double tolerance) {
    const curve_surface_intersection::ParameterMatch proof =
        curve_surface_intersection::same_parameter(world, pc, surface, tolerance);
    if (proof != curve_surface_intersection::ParameterMatch::Matched)
        throw brep_assemble::ParameterError("Native hollow rim lacks a same-parameter certificate",
            world, pc, surface, tolerance, proof);
}

NurbsCurve normalized_uv(NurbsCurve curve, const NurbsSurface& support) {
    const std::pair<double, double> u = support.domain(0), v = support.domain(1);
    require(std::isfinite(u.second - u.first) && u.second > u.first
        && std::isfinite(v.second - v.first) && v.second > v.first,
        "Native hollow rim has an invalid parameter chart");
    for (int i = 0; i < curve.cv_count(); ++i) {
        curve.cv(i)[0] = (curve.cv(i)[0] - u.first * curve.weight(i)) / (u.second - u.first);
        curve.cv(i)[1] = (curve.cv(i)[1] - v.first * curve.weight(i)) / (v.second - v.first);
        if (curve.dimension() > 2)
            curve.cv(i)[2] = 0;
    }
    return curve;
}

double chart_tolerance(const NurbsSurface& support, double tolerance) {
    Point low = support.get_cv(0, 0), high = low;
    for (int i = 0; i < support.cv_count(0); ++i)
        for (int j = 0; j < support.cv_count(1); ++j)
            for (int d = 0; d < 3; ++d) {
                low[d] = std::min(low[d], support.get_cv(i, j)[d]);
                high[d] = std::max(high[d], support.get_cv(i, j)[d]);
            }
    const double diameter = low.distance(high);
    require(std::isfinite(diameter) && diameter > 0, "Native hollow mouth has a collapsed control hull");
    const double result = tolerance * .0625 / diameter;
    require(std::isfinite(result) && result > 64 * std::numeric_limits<double>::epsilon(),
        "Native hollow rim separation is below chart resolution");
    return result;
}

void nested_rim(const BRep& source, int opening, const BRep& inner, int image, double tolerance) {
    const NurbsSurface& support = source.m_surfaces[source.m_faces[opening].surface_index];
    require(inner.m_faces[image].wires.size() == 1, "Native hollow opening image requires one outer wire");
    require(nurbs_coincidence::same_parameters(support, inner.m_surfaces[inner.m_faces[image].surface_index], tolerance),
        "Native hollow opening image changed its parameter chart");
    require(source.face_orientation(opening) == inner.face_orientation(image),
        "Native hollow opening image changed its oriented support");
    const std::vector<BRepRef> outer = source.wire_edges(source.m_faces[opening].wires[0]);
    const std::vector<BRepRef> inside = inner.wire_edges(inner.m_faces[image].wires[0]);
    require(!outer.empty() && !inside.empty(), "Native hollow rim has an empty wire");
    const double uv_tolerance = chart_tolerance(support, tolerance);
    BRep normalized = source;
    for (NurbsCurve& curve : normalized.m_curves_2d)
        curve = normalized_uv(curve, support);
    for (const BRepRef& use : inside) {
        const int pc = inner.pcurve_index(use.index, image, use.orientation);
        require(pc >= 0, "Native hollow opening image lacks its pcurve");
        const NurbsCurve& curve = inner.m_curves_2d[pc];
        require(brep_classify::face_uv(normalized, opening, normalized_uv(curve, support).point_at_start(), uv_tolerance)
            == brep_classify::State::Inside, "Native hollow opening image is not strictly inside the source mouth");
        for (const BRepRef& boundary : outer) {
            const int first = source.pcurve_index(boundary.index, opening, boundary.orientation);
            require(first >= 0, "Native hollow source opening lacks its pcurve");
            const curve_intersection::Result crossings = curve_intersection::intersect(
                normalized_uv(source.m_curves_2d[first], support), normalized_uv(curve, support), uv_tolerance);
            require(crossings.complete && crossings.points.empty() && crossings.overlaps.empty(),
                "Native hollow rim boundaries intersect or their separation is unresolved");
        }
    }
}

struct Rim {
    BRep brep;
    std::vector<split::EdgePiece> pieces;
};

int append_loop(Rim& rim, const BRep& source, int face, int identity_base, double tolerance) {
    std::map<int, int> vertices;
    std::vector<BRepRef> wire;
    const NurbsSurface& support = rim.brep.m_surfaces[0];
    for (const BRepRef& use : source.wire_edges(source.m_faces[face].wires[0])) {
        const BRepEdge& original = source.m_edges[use.index];
        for (int vertex : {original.start_vertex, original.end_vertex})
            if (!vertices.count(vertex))
                vertices[vertex] = rim.brep.add_vertex(source.m_vertices[vertex].point);
        const int index = source.pcurve_index(use.index, face, use.orientation);
        require(index >= 0, "Native hollow rim edge lacks a pcurve");
        const NurbsCurve& pc = source.m_curves_2d[index];
        NurbsCurve world = source.m_curves_3d[original.curve_3d_index];
        require(world.set_domain(pc.domain_start(), pc.domain_end()), "Native hollow rim could not synchronize parameters");
        certify(world, pc, support, tolerance);
        const int edge = rim.brep.add_edge(rim.brep.add_curve_3d(world), vertices[original.start_vertex], vertices[original.end_vertex]);
        rim.brep.add_pcurve(edge, 0, rim.brep.add_curve_2d(pc));
        rim.pieces.push_back({-1, identity_base + use.index, edge, world.domain_start(), world.domain_end()});
        wire.push_back({edge, use.orientation});
    }
    return rim.brep.add_wire(wire);
}

Rim opening_rim(const BRep& source, int opening, const BRep& inner, int image, double tolerance) {
    nested_rim(source, opening, inner, image, tolerance);
    Rim result;
    result.brep.add_surface(source.m_surfaces[source.m_faces[opening].surface_index]);
    const int outer = append_loop(result, source, opening, 0, tolerance);
    const int inside = append_loop(result, inner, image, source.edge_count(), tolerance);
    result.brep.add_face(0, {{outer, BRepOrientation::Forward}, {inside, BRepOrientation::Reversed}}, tolerance);
    result.brep.add_shell({{0, source.face_orientation(opening)}});
    require(result.brep.is_valid(), "Native hollow rim construction has invalid topology");
    return result;
}
}

BRep shape(const BRep& source, const std::vector<int>& openings, double thickness, double tolerance) {
    const std::set<int> removed = validate(source, openings, thickness, tolerance);
    if (removed.empty())
        return brep_offset::shape(source, thickness, tolerance);
    std::set<int> context_edges;
    for (int face : removed)
        for (const BRepRef& edge : source.wire_edges(source.m_faces[face].wires[0]))
            context_edges.insert(edge.index);
    std::vector<brep_offset::OffsetFace> supports;
    for (int face = 0; face < source.face_count(); ++face) {
        brep_offset::OffsetFace working = brep_offset::displaced_face(source, face, 0, tolerance * .125);
        bool adjacent = false;
        for (const std::pair<const int, int>& edge : working.edges)
            if (context_edges.count(edge.first)) {
                adjacent = true;
                break;
            }
        const double signed_distance = source.face_orientation(face) == BRepOrientation::Reversed ? -thickness : thickness;
        if (!removed.count(face) && adjacent
            && source_working::prepare(source, working, signed_distance, tolerance * .125))
            supports.push_back(brep_offset::displaced_working_face(source, face, working, thickness, tolerance * .125));
        else if (removed.count(face))
            supports.push_back(std::move(working));
        else
            supports.push_back(brep_offset::displaced_face(source, face, thickness, tolerance * .125));
    }
    for (brep_offset::OffsetFace& support : supports) {
        if (removed.count(support.source))
            continue;
        bool adjacent = false;
        for (const std::pair<const int, int>& edge : support.edges)
            if (context_edges.count(edge.first)) {
                adjacent = true;
                break;
            }
        if (adjacent)
            affine_working::enlarge(source, support, thickness, tolerance * .125);
    }
    brep_offset::InwardResult displaced = brep_offset::inward_with_ancestry(source, std::move(supports), thickness, tolerance,
        std::vector<int>(removed.begin(), removed.end()));
    const BRep& inner = displaced.brep;
    require(displaced.source_faces.size() == inner.m_faces.size(), "Native hollow lost displaced face ancestry");
    std::map<int, int> images;
    for (int face = 0; face < inner.face_count(); ++face)
        if (removed.count(displaced.source_faces[face]))
            require(images.emplace(displaced.source_faces[face], face).second, "Native hollow opening has multiple offset images");
    require(images.size() == removed.size(), "Native hollow lost an opening image");
    std::vector<Rim> rims;
    for (int face : removed)
        rims.push_back(opening_rim(source, face, inner, images.at(face), tolerance));
    const std::vector<split::EdgePiece> source_edges = identities(source, 0);
    const std::vector<split::EdgePiece> inner_edges = identities(inner, source.edge_count());
    std::vector<brep_assemble::FaceUse> selected;
    for (int face = 0; face < source.face_count(); ++face)
        if (!removed.count(face))
            selected.push_back({&source, face, false, &source_edges, 0});
    for (int face = 0; face < inner.face_count(); ++face)
        if (!removed.count(displaced.source_faces[face]))
            selected.push_back({&inner, face, true, &inner_edges, 1});
    for (const Rim& rim : rims)
        selected.push_back({&rim.brep, 0, false, &rim.pieces, 2});
    BRep result = brep_assemble::assemble(selected, tolerance);
    require(result.is_valid() && result.is_solid() && result.solid_count() == 1,
        "Native hollowing did not reconstruct one closed solid");
    require(result.shell_count() == 1, "Native hollowing changed expected shell ownership");
    result.name = source.name;
    result.surfacecolor = source.surfacecolor;
    result.is_visible = source.is_visible;
    result.is_locked = source.is_locked;
    result.width = source.width;
    return result;
}
}

namespace session_cpp {
BRep BRep::hollow(const std::vector<int>& openings, double thickness, double tolerance) const {
    return brep_hollow::shape(*this, openings, thickness, tolerance);
}
}
