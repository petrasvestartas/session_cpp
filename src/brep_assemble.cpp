#include "brep_assemble.h"
#include "brep_classify.h"
#include "curve_intersection.h"
#include "curve_surface_intersection.h"
#include "split.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace session_cpp::brep_assemble {
namespace {

constexpr double epsilon = std::numeric_limits<double>::epsilon();
constexpr BRepOrientation forward = BRepOrientation::Forward;
constexpr BRepOrientation reversed = BRepOrientation::Reversed;

struct Bounds {
    Point lo;
    Point hi;

    explicit Bounds(const NurbsCurve& curve) : lo(curve.get_cv(0)), hi(lo) {
        for (int i = 1; i < curve.cv_count(); ++i) {
            const Point p = curve.get_cv(i);
            for (int d = 0; d < 3; ++d) {
                lo[d] = std::min(lo[d], p[d]);
                hi[d] = std::max(hi[d], p[d]);
            }
        }
    }

    bool overlaps(const Bounds& box, double tolerance) const {
        for (int d = 0; d < 3; ++d)
            if (hi[d] + tolerance < box.lo[d] || box.hi[d] + tolerance < lo[d])
                return false;

        return true;
    }
};

struct Piece {
    int edge;
    double first;
    double last;
    bool reversed;
};

struct ParameterRelation {
    int pcurve;
    int surface;
    bool matched;
};

struct Source {
    const BRep* brep;
    int edge;
    int operand = -1;
    int original = -1;
    int section = -1;
    std::vector<double> cuts;
    std::vector<Piece> pieces;
    std::vector<ParameterRelation> parameters;
};

struct Surface {
    const BRep* brep;
    int source;
    int target;
};

struct Canonical {
    size_t source;
    int edge;
};

struct PcurveUse {
    int edge;
    int surface;
    BRepOrientation orientation;
};

struct Assembly {
    BRep result;
    std::vector<Source> sources;
    std::vector<Surface> surfaces;
    std::vector<Canonical> canonical;
    std::vector<PcurveUse> pcurves;
    double tolerance;
};

void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error("BRep assembly: " + message);
}

const NurbsCurve& world(const Source& source) {
    const BRepEdge& edge = source.brep->m_edges[source.edge];
    return source.brep->m_curves_3d[edge.curve_3d_index];
}

bool degenerated(const Source& source) {
    return source.brep->m_edges[source.edge].degenerated;
}

size_t source_index(const Assembly& assembly, const BRep* brep, int edge) {
    for (size_t i = 0; i < assembly.sources.size(); ++i)
        if (assembly.sources[i].brep == brep && assembly.sources[i].edge == edge)
            return i;

    return assembly.sources.size();
}

void add_source(Assembly& assembly, const FaceUse& face, int edge) {
    if (source_index(assembly, face.brep, edge) != assembly.sources.size())
        return;

    Source source{face.brep, edge};
    source.operand = face.operand;
    if (face.pieces)
        for (const split::EdgePiece& piece : *face.pieces)
            if (piece.edge == edge) {
                source.original = piece.source_edge;
                source.section = piece.section;
                break;
            }

    if (!degenerated(source)) {
        const NurbsCurve& curve = world(source);
        source.cuts = {curve.domain_start(), curve.domain_end()};
    }
    assembly.sources.push_back(std::move(source));
}

void collect(Assembly& assembly, const std::vector<FaceUse>& faces) {
    for (size_t i = 0; i < faces.size(); ++i) {
        const FaceUse& face = faces[i];
        require(face.brep && face.brep->is_valid(), "invalid source BRep");
        require(face.face >= 0 && face.face < face.brep->face_count(), "invalid selected face");
        for (size_t j = 0; j < i; ++j)
            require(faces[j].brep != face.brep || faces[j].face != face.face, "duplicate selected face");

        for (const BRepRef& wire : face.brep->m_faces[face.face].wires)
            for (const BRepRef& edge : face.brep->wire_edges(wire)) {
                require(edge.orientation == forward || edge.orientation == reversed, "non-boundary edge use");
                add_source(assembly, face, edge.index);
            }
    }
}

void synchronize(Assembly& assembly) {
    size_t work = 0;
    for (size_t i = 0; i < assembly.sources.size(); ++i) {
        Source& a = assembly.sources[i];
        if (degenerated(a))
            continue;

        const Bounds bounds(world(a));
        for (size_t j = i + 1; j < assembly.sources.size(); ++j) {
            require(++work < 2000000, "edge interference work limit exceeded");
            Source& b = assembly.sources[j];
            if (degenerated(b) || !bounds.overlaps(Bounds(world(b)), assembly.tolerance))
                continue;

            const curve_intersection::Result intersections = curve_intersection::intersect(world(a), world(b), assembly.tolerance);
            require(intersections.complete, intersections.failure + " in source edge pair " + std::to_string(a.edge)
                + "," + std::to_string(b.edge) + "; operand " + std::to_string(a.operand) + "," + std::to_string(b.operand)
                + "; hull diameters " + std::to_string(bounds.lo.distance(bounds.hi))
                + "," + std::to_string(Bounds(world(b)).lo.distance(Bounds(world(b)).hi)));
            for (const curve_intersection::Hit& point : intersections.points) {
                a.cuts.push_back(point.a);
                b.cuts.push_back(point.b);
            }
            for (const curve_intersection::Overlap& shared : intersections.overlaps) {
                a.cuts.push_back(shared.a0);
                a.cuts.push_back(shared.a1);
                b.cuts.push_back(shared.b0);
                b.cuts.push_back(shared.b1);
            }
        }
    }
}

bool within_vertex(const NurbsCurve& curve, double a, double b, double tolerance) {
    NurbsCurve part = curve;
    require(part.trim(a, b), "could not inspect a vertex-sized edge interval");
    const Point center = curve.point_at(a);
    for (int i = 0; i < part.cv_count(); ++i) {
        const Point point = part.get_cv(i);
        const double magnitude = std::max({std::abs(point[0]), std::abs(point[1]), std::abs(point[2])});
        if (point.distance(center) + 128 * epsilon * magnitude > tolerance)
            return false;
    }
    return true;
}

std::vector<double> sorted_cuts(const Source& source, double tolerance) {
    std::vector<double> sorted = source.cuts;
    std::sort(sorted.begin(), sorted.end());
    std::vector<double> cuts;
    const NurbsCurve& curve = world(source);
    const double scale = std::max({std::abs(curve.domain_start()), std::abs(curve.domain_end()),
        curve.domain_end() - curve.domain_start()});
    for (const double t : sorted)
        if (cuts.empty() || t - cuts.back() > epsilon * scale * 128.0)
            cuts.push_back(t);

    cuts.front() = curve.domain_start();
    cuts.back() = curve.domain_end();
    std::vector<double> vertices{cuts.front()};
    for (size_t i = 1; i < cuts.size(); ++i) {
        if (within_vertex(curve, vertices.back(), cuts[i], tolerance * 0.25)) {
            if (i + 1 == cuts.size()) {
                require(vertices.size() > 1, "edge lies entirely inside its vertex tolerance");
                vertices.back() = cuts[i];
            }
        } else {
            vertices.push_back(cuts[i]);
        }
    }
    return vertices;
}

NurbsCurve interval(const NurbsCurve& curve, double a, double b) {
    NurbsCurve result = curve;
    require(b > a, "collapsed curve interval");
    if (a > curve.domain_start() || b < curve.domain_end())
        require(result.trim(a, b), "could not trim an edge interval");

    return result;
}

int vertex(Assembly& assembly, const Point& point) {
    for (int i = 0; i < assembly.result.vertex_count(); ++i)
        if (assembly.result.m_vertices[i].point.distance(point) <= assembly.tolerance)
            return i;

    return assembly.result.add_vertex(point, assembly.tolerance);
}

bool same_identity(const Source& a, const Source& b) {
    if (a.brep == b.brep && a.edge == b.edge)
        return true;
    if (a.section >= 0 && a.section == b.section)
        return true;

    return a.operand >= 0 && a.operand == b.operand && a.original >= 0 && a.original == b.original;
}

bool covers(double a, double b, const NurbsCurve& curve) {
    const double span = curve.domain_end() - curve.domain_start();
    return std::abs(a - curve.domain_start()) <= span * 1e-10 &&
        std::abs(b - curve.domain_end()) <= span * 1e-10;
}

bool equivalent(const NurbsCurve& a, const NurbsCurve& b, double tolerance, bool& reverse) {
    const bool along = a.point_at_start().distance(b.point_at_start()) <= tolerance &&
        a.point_at_end().distance(b.point_at_end()) <= tolerance;
    const bool against = a.point_at_start().distance(b.point_at_end()) <= tolerance &&
        a.point_at_end().distance(b.point_at_start()) <= tolerance;
    if (!along && !against)
        return false;

    const curve_intersection::Result result = curve_intersection::intersect(a, b, tolerance);
    require(result.complete, result.failure + " while matching representative pieces; domains "
        + std::to_string(a.domain_start()) + "," + std::to_string(a.domain_end()) + " and "
        + std::to_string(b.domain_start()) + "," + std::to_string(b.domain_end())
        + "; hull diameters " + std::to_string(Bounds(a).lo.distance(Bounds(a).hi))
        + "," + std::to_string(Bounds(b).lo.distance(Bounds(b).hi)));
    for (const curve_intersection::Overlap& shared : result.overlaps)
        if (covers(shared.a0, shared.a1, a) && covers(std::min(shared.b0, shared.b1), std::max(shared.b0, shared.b1), b)) {
            reverse = shared.b1 < shared.b0;
            return true;
        }

    return false;
}

Piece add_piece(Assembly& assembly, size_t index, double a, double b) {
    const Source& source = assembly.sources[index];
    const NurbsCurve curve = interval(world(source), a, b);
    for (int pass = 0; pass < 2; ++pass)
        for (const Canonical& candidate : assembly.canonical) {
            const bool identity = same_identity(source, assembly.sources[candidate.source]);
            if (identity != (pass == 0))
                continue;

            const BRepEdge& edge = assembly.result.m_edges[candidate.edge];
            const NurbsCurve& stored = assembly.result.m_curves_3d[edge.curve_3d_index];
            bool reverse = false;
            if (equivalent(curve, stored, assembly.tolerance, reverse))
                return {candidate.edge, a, b, reverse};
        }

    const int first = vertex(assembly, curve.point_at_start());
    const int last = vertex(assembly, curve.point_at_end());
    const double tolerance = std::max(assembly.tolerance, source.brep->m_edges[source.edge].tolerance);
    const int edge = assembly.result.add_edge(assembly.result.add_curve_3d(curve), first, last, tolerance);
    assembly.canonical.push_back({index, edge});
    return {edge, a, b, false};
}

void partition(Assembly& assembly) {
    for (size_t i = 0; i < assembly.sources.size(); ++i) {
        Source& source = assembly.sources[i];
        if (degenerated(source)) {
            const BRepEdge& original = source.brep->m_edges[source.edge];
            const Point point = source.brep->m_vertices[original.start_vertex].point;
            const int v = vertex(assembly, point);
            const int edge = assembly.result.add_edge(-1, v, v, assembly.tolerance);
            source.pieces.push_back({edge, 0.0, 1.0, false});
            continue;
        }

        const std::vector<double> cuts = sorted_cuts(source, assembly.tolerance);
        for (size_t j = 1; j < cuts.size(); ++j)
            source.pieces.push_back(add_piece(assembly, i, cuts[j - 1], cuts[j]));
    }
}

int surface_index(Assembly& assembly, const FaceUse& face) {
    const int index = face.brep->m_faces[face.face].surface_index;
    for (const Surface& surface : assembly.surfaces)
        if (surface.brep == face.brep && surface.source == index)
            return surface.target;

    const int target = assembly.result.add_surface(face.brep->m_surfaces[index]);
    assembly.surfaces.push_back({face.brep, index, target});
    return target;
}

Point lifted(const NurbsSurface& surface, const NurbsCurve& curve, double t) {
    const Point uv = curve.point_at(t);
    return surface.point_at(uv[0], uv[1]);
}

double pcurve_parameter(
    const NurbsSurface& surface,
    const NurbsCurve& curve,
    const Point& point,
    double guess,
    double tolerance
) {
    double t = guess;
    double gap = lifted(surface, curve, t).distance(point);
    for (int i = 0; i <= 16; ++i) {
        const double sample = curve.domain_start() + (curve.domain_end() - curve.domain_start()) * i / 16.0;
        const double distance = lifted(surface, curve, sample).distance(point);
        if (distance < gap) {
            t = sample;
            gap = distance;
        }
    }
    for (int i = 0; i < 48; ++i) {
        const std::vector<Vector> pc = curve.evaluate(t, 1);
        const std::vector<Vector> values = surface.evaluate(pc[0][0], pc[0][1], 1);
        const Vector tangent = values[2] * pc[1][0] + values[1] * pc[1][1];
        const Vector delta = lifted(surface, curve, t) - point;
        const double length2 = tangent.dot(tangent);
        if (length2 == 0.0)
            break;

        const double next = std::clamp(t - delta.dot(tangent) / length2, curve.domain_start(), curve.domain_end());
        if (next == t)
            break;

        t = next;
    }
    require(lifted(surface, curve, t).distance(point) <= tolerance, "edge and lifted pcurve do not agree");
    return t;
}

bool source_parameterized(
    Source& source,
    int pcurve,
    int surface,
    double tolerance
) {
    for (const auto& relation : source.parameters)
        if (relation.pcurve == pcurve && relation.surface == surface)
            return relation.matched;
    const bool matched = curve_surface_intersection::same_parameter(world(source), source.brep->m_curves_2d[pcurve],
        source.brep->m_surfaces[surface], tolerance) == curve_surface_intersection::ParameterMatch::Matched;
    source.parameters.push_back({pcurve, surface, matched});
    return matched;
}

double corresponding_parameter(const NurbsCurve& world, const NurbsCurve& pcurve, double parameter) {
    const double first = world.domain_start();
    const double last = world.domain_end();
    const double start = pcurve.domain_start();
    const double end = pcurve.domain_end();
    if (first == start && last == end)
        return parameter;
    if (parameter == first)
        return start;
    if (parameter == last)
        return end;
    return start + (parameter - first) / (last - first) * (end - start);
}

NurbsCurve piece_pcurve(
    Source& source,
    const FaceUse& face,
    const BRepRef& use,
    const Piece& piece,
    double tolerance
) {
    const int index = source.brep->pcurve_index(source.edge, face.face, use.orientation);
    require(index >= 0, "selected face has no edge pcurve");
    NurbsCurve result = source.brep->m_curves_2d[index];
    if (!degenerated(source)) {
        const NurbsCurve& curve = world(source);
        const int surface_index = source.brep->m_faces[face.face].surface_index;
        const NurbsSurface& surface = source.brep->m_surfaces[surface_index];
        const double lo = result.domain_start();
        const double ga = corresponding_parameter(curve, result, piece.first);
        const double gb = corresponding_parameter(curve, result, piece.last);
        const bool matched = source_parameterized(source, index, surface_index, tolerance);
        const double a = piece.first == curve.domain_start() ? lo :
            (matched ? ga : pcurve_parameter(surface, result, curve.point_at(piece.first), ga, tolerance));
        const double b = piece.last == curve.domain_end() ? result.domain_end() :
            (matched ? gb : pcurve_parameter(surface, result, curve.point_at(piece.last), gb, tolerance));
        require(b > a, "pcurve partition crosses an unresolved parameter seam");
        result = interval(result, a, b);
    }
    if (piece.reversed)
        require(result.reverse(), "could not reverse a shared edge pcurve");

    return result;
}

bool same_uv_trace(const NurbsCurve& a, const NurbsCurve& b) {
    const Bounds first(a);
    const Bounds second(b);
    const double scale = std::max({first.lo.distance(first.hi), second.lo.distance(second.hi),
        std::abs(first.lo[0]), std::abs(first.lo[1]), std::abs(first.hi[0]), std::abs(first.hi[1]),
        std::abs(second.lo[0]), std::abs(second.lo[1]), std::abs(second.hi[0]), std::abs(second.hi[1])});
    bool reverse = false;
    return equivalent(a, b, epsilon * scale * 512.0, reverse) && !reverse;
}

std::string parameter_message(
    const std::string& reason,
    const NurbsCurve& curve,
    const NurbsCurve& pcurve,
    const NurbsSurface& surface,
    double tolerance
) {
    std::ostringstream output;
    output.precision(17);
    output << reason << "; tolerance=" << tolerance << "; world=" << curve.cv_count() << " CVs ["
        << curve.domain_start() << ',' << curve.domain_end() << "]; pcurve=" << pcurve.cv_count()
        << " CVs [" << pcurve.domain_start() << ',' << pcurve.domain_end() << "]; surface="
        << surface.cv_count(0) << 'x' << surface.cv_count(1) << " CVs";
    return output.str();
}

NurbsCurve parameterized_pcurve(
    const NurbsCurve& world,
    NurbsCurve pcurve,
    const NurbsSurface& surface,
    double tolerance
) {
    const curve_surface_intersection::ParameterMatch match =
        curve_surface_intersection::same_parameter(world, pcurve, surface, tolerance);
    if (match == curve_surface_intersection::ParameterMatch::Mismatched) {
        const curve_surface_intersection::Result result = curve_surface_intersection::intersect(world, surface, tolerance);
        if (!result.complete)
            throw ParameterError("pcurve reparameterization is unresolved: " + result.failure,
                world, pcurve, surface, tolerance, match);
        bool repaired = false;
        for (const curve_surface_intersection::Overlap& overlap : result.overlaps)
            if (covers(overlap.t0, overlap.t1, world) && same_uv_trace(pcurve, overlap.pcurve) &&
                curve_surface_intersection::same_parameter(world, overlap.pcurve, surface, tolerance) ==
                    curve_surface_intersection::ParameterMatch::Matched) {
                pcurve = overlap.pcurve;
                repaired = true;
                break;
            }
        if (!repaired)
            throw ParameterError("edge and pcurve disagree inside their ranges", world, pcurve, surface, tolerance, match);
    } else if (match != curve_surface_intersection::ParameterMatch::Matched)
        throw ParameterError("pcurve same-parameter certification is unresolved", world, pcurve, surface, tolerance, match);
    require(pcurve.set_domain(world.domain_start(), world.domain_end()), "pcurve same-range conversion failed");
    return pcurve;
}

void attach_pcurve(
    Assembly& assembly,
    int edge,
    int surface,
    BRepOrientation orientation,
    const NurbsCurve& curve
) {
    BRepEdge& boundary = assembly.result.m_edges[edge];
    NurbsCurve pcurve = curve;
    if (!boundary.degenerated)
        pcurve = parameterized_pcurve(assembly.result.m_curves_3d[boundary.curve_3d_index],
            pcurve, assembly.result.m_surfaces[surface], boundary.tolerance);
    for (BRepCurveOnSurface& record : boundary.pcurves) {
        if (record.surface_index != surface)
            continue;

        for (const PcurveUse& first : assembly.pcurves)
            if (first.edge == edge && first.surface == surface) {
                if (record.curve_2d_index_2 >= 0 || first.orientation == orientation) {
                    const int previous = record.curve_2d_index_2 >= 0 && orientation == reversed ?
                        record.curve_2d_index_2 : record.curve_2d_index;
                    require(same_uv_trace(assembly.result.m_curves_2d[previous], pcurve),
                        "distinct pcurve branches share one edge orientation");
                    return;
                }

                const int index = assembly.result.add_curve_2d(pcurve);
                if (orientation == forward) {
                    record.curve_2d_index_2 = record.curve_2d_index;
                    record.curve_2d_index = index;
                } else {
                    record.curve_2d_index_2 = index;
                }
                return;
            }
    }
    assembly.result.add_pcurve(edge, surface, assembly.result.add_curve_2d(pcurve));
    assembly.pcurves.push_back({edge, surface, orientation});
}

void append_edge(
    Assembly& assembly,
    const FaceUse& face,
    const BRepRef& use,
    const Piece& piece,
    int surface,
    std::vector<BRepRef>& edges
) {
    Source& source = assembly.sources[source_index(assembly, face.brep, use.index)];
    const BRepOrientation orientation = piece.reversed ? brep_reverse(use.orientation) : use.orientation;
    const NurbsCurve pc = piece_pcurve(source, face, use, piece, assembly.tolerance);
    attach_pcurve(assembly, piece.edge, surface, orientation, pc);
    edges.push_back({piece.edge, orientation});
}

void copy_face(Assembly& assembly, const FaceUse& face) {
    const int surface = surface_index(assembly, face);
    std::vector<BRepRef> wires;
    for (const BRepRef& wire : face.brep->m_faces[face.face].wires) {
        std::vector<BRepRef> edges;
        for (const BRepRef& use : face.brep->wire_edges(wire)) {
            const Source& source = assembly.sources[source_index(assembly, face.brep, use.index)];
            if (use.orientation == forward) {
                for (const Piece& piece : source.pieces)
                    append_edge(assembly, face, use, piece, surface, edges);
            } else {
                for (size_t i = source.pieces.size(); i-- > 0;)
                    append_edge(assembly, face, use, source.pieces[i], surface, edges);
            }
        }
        wires.push_back({assembly.result.add_wire(edges), forward});
    }
    const BRepFace& original = face.brep->m_faces[face.face];
    const int index = assembly.result.add_face(surface, wires, std::max(assembly.tolerance, original.tolerance));
    assembly.result.m_faces[index].facecolor = original.facecolor;
}

std::vector<std::vector<BRepRef>> edge_uses(const BRep& brep, const std::vector<BRepOrientation>& orientations) {
    std::vector<std::vector<BRepRef>> uses(brep.edge_count());
    for (int face = 0; face < brep.face_count(); ++face)
        for (const BRepRef& wire : brep.m_faces[face].wires)
            for (const BRepRef& edge : brep.wire_edges(wire))
                if (!brep.m_edges[edge.index].degenerated)
                    uses[edge.index].push_back({face, brep_compose(orientations[face], edge.orientation)});

    for (int i = 0; i < brep.edge_count(); ++i) {
        if (brep.m_edges[i].degenerated)
            continue;

        int balance = 0;
        for (const BRepRef& use : uses[i])
            balance += use.orientation == forward ? 1 : -1;
        require(uses[i].size() >= 2 && balance == 0, "open or inconsistently oriented edge " + std::to_string(i));
    }
    return uses;
}

Vector face_binormal(const BRep& brep, int edge, const BRepRef& use,
                     const std::vector<BRepOrientation>& orientations, const Vector& tangent) {
    const BRepOrientation local = brep_compose(orientations[use.index], use.orientation);
    const int index = brep.pcurve_index(edge, use.index, local);
    require(index >= 0, "missing pcurve at a multi-face edge");
    const NurbsCurve& curve = brep.m_curves_2d[index];
    const Point uv = curve.point_at_middle();
    const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[use.index].surface_index];
    const auto derivatives = surface.evaluate(uv[0], uv[1], 1);
    require(derivatives.size() == 3, "undefined normal at a multi-face edge");
    Vector normal = derivatives[2].cross(derivatives[1]);
    require(normal.normalize_self(), "singular normal at a multi-face edge");
    if (orientations[use.index] == reversed)
        normal *= -1.0;
    Vector binormal = normal.cross(use.orientation == forward ? tangent : tangent * -1.0);
    require(binormal.normalize_self(), "undefined face direction at a multi-face edge");
    return binormal;
}

std::vector<int> edge_mates(const BRep& brep, int edge, const std::vector<BRepRef>& uses,
                           const std::vector<BRepOrientation>& orientations) {
    if (uses.size() == 2)
        return {1, 0};
    const NurbsCurve& curve = brep.m_curves_3d[brep.m_edges[edge].curve_3d_index];
    Vector tangent = curve.evaluate(curve.domain_middle(), 1)[1];
    require(tangent.normalize_self(), "singular tangent at a multi-face edge");
    std::vector<Vector> directions;
    for (const BRepRef& use : uses)
        directions.push_back(face_binormal(brep, edge, use, orientations, tangent));
    const double pi = std::acos(-1.0);
    std::vector<int> mates(uses.size(), -1);
    for (size_t i = 0; i < uses.size(); ++i) {
        double best = 2 * pi + 1, next = best;
        const Vector axis = uses[i].orientation == forward ? tangent * -1.0 : tangent;
        for (size_t j = 0; j < uses.size(); ++j) {
            if (uses[i].orientation == uses[j].orientation)
                continue;
            double angle = std::atan2(axis.dot(directions[i].cross(directions[j])), directions[i].dot(directions[j]));
            if (angle < 0)
                angle += 2 * pi;
            if (angle < best) {
                next = best;
                best = angle;
                mates[i] = static_cast<int>(j);
            } else {
                next = std::min(next, angle);
            }
        }
        require(mates[i] >= 0 && best > 1e-7 && next - best > 1e-7,
            "ambiguous tangent faces at edge " + std::to_string(edge));
    }
    for (size_t i = 0; i < mates.size(); ++i)
        require(mates[mates[i]] == static_cast<int>(i), "inconsistent angular face pairing");
    return mates;
}

void build_shells(BRep& brep, const std::vector<BRepOrientation>& orientations) {
    const std::vector<std::vector<BRepRef>> uses = edge_uses(brep, orientations);
    std::vector<std::vector<int>> mates(brep.edge_count());
    for (int edge = 0; edge < brep.edge_count(); ++edge)
        if (!brep.m_edges[edge].degenerated)
            mates[edge] = edge_mates(brep, edge, uses[edge], orientations);
    std::vector<bool> visited(brep.face_count(), false);
    for (int seed = 0; seed < brep.face_count(); ++seed) {
        if (visited[seed])
            continue;

        std::vector<int> queue{seed};
        std::vector<BRepRef> faces;
        visited[seed] = true;
        for (size_t i = 0; i < queue.size(); ++i) {
            const int face = queue[i];
            faces.push_back({face, orientations[face]});
            for (const BRepRef& wire : brep.m_faces[face].wires)
                for (const BRepRef& edge : brep.wire_edges(wire))
                    for (size_t use = 0; use < uses[edge.index].size(); ++use) {
                        if (uses[edge.index][use].index != face)
                            continue;
                        const int neighbour = uses[edge.index][mates[edge.index][use]].index;
                        if (!visited[neighbour]) {
                            visited[neighbour] = true;
                            queue.push_back(neighbour);
                        }
                    }
        }
        const int shell = brep.add_shell(faces);
        require(brep.is_closed(shell), "angular face pairing produced an open or nonmanifold shell");
        brep.add_solid({{shell, forward}});
    }
}

bool interior_uv(const BRep& brep, int face, const Point& uv) {
    const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
    const std::pair<double, double> u = surface.domain(0);
    const std::pair<double, double> v = surface.domain(1);
    const double scale = std::max({std::abs(u.first), std::abs(u.second), std::abs(v.first),
        std::abs(v.second), u.second - u.first, v.second - v.first});
    return brep_classify::face_uv(brep, face, uv, epsilon * scale * 128.0) == brep_classify::State::Inside;
}

Point face_parameter(const BRep& brep, int face) {
    const BRepFace& boundary = brep.m_faces[face];
    const NurbsSurface& surface = brep.m_surfaces[boundary.surface_index];
    const std::pair<double, double> u = surface.domain(0);
    const std::pair<double, double> v = surface.domain(1);
    for (int i = 1; i < 8; ++i)
        for (int j = 1; j < 8; ++j) {
            const Point uv(u.first + (u.second - u.first) * i / 8.0,
                v.first + (v.second - v.first) * j / 8.0, 0.0);
            if (interior_uv(brep, face, uv))
                return uv;
        }
    const double scale = std::max(u.second - u.first, v.second - v.first);
    for (const BRepRef& edge : brep.wire_edges(boundary.wires.front())) {
        const int index = brep.pcurve_index(edge.index, face, edge.orientation);
        require(index >= 0, "missing probe pcurve");
        const NurbsCurve& curve = brep.m_curves_2d[index];
        const Point midpoint = curve.point_at_middle();
        const Vector tangent = curve.evaluate(curve.domain_middle(), 1)[1];
        Vector normal(-tangent[1], tangent[0], 0.0);
        if (!normal.normalize_self())
            continue;

        double distance = scale * 0.1;
        for (int i = 0; i < 10; ++i) {
            for (const double sign : {-1.0, 1.0}) {
                const Point uv = midpoint + normal * (distance * sign);
                if (interior_uv(brep, face, uv))
                    return uv;
            }
            distance *= 0.1;
        }
    }
    throw std::runtime_error("BRep assembly: could not find an interior face parameter");
}

BRep shell_solid(const BRep& brep, int shell) {
    BRep result = brep;
    result.m_solids.clear();
    result.add_solid({{shell, forward}});
    return result;
}

Point shell_point(const BRep& brep, int shell) {
    const int face = brep.m_shells[shell].faces.front().index;
    const Point uv = face_parameter(brep, face);
    return brep.m_surfaces[brep.m_faces[face].surface_index].point_at(uv[0], uv[1]);
}

bool outward(const BRep& brep, int shell, double tolerance) {
    const BRep region = shell_solid(brep, shell);
    for (const BRepRef& face : brep.m_shells[shell].faces) {
        const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face.index].surface_index];
        const Point uv = face_parameter(brep, face.index);
        const Point point = surface.point_at(uv[0], uv[1]);
        Vector normal = surface.normal_at(uv[0], uv[1]);
        if (face.orientation == reversed)
            normal = -normal;
        if (!normal.normalize_self())
            continue;

        double step = tolerance * 64.0;
        for (int i = 0; i < 5; ++i) {
            const brep_classify::State plus = brep_classify::point(region, point + normal * step, tolerance);
            const brep_classify::State minus = brep_classify::point(region, point - normal * step, tolerance);
            if (plus == brep_classify::State::Outside && minus == brep_classify::State::Inside)
                return true;
            if (plus == brep_classify::State::Inside && minus == brep_classify::State::Outside)
                return false;

            step *= 2.0;
        }
    }
    throw std::runtime_error("BRep assembly: shell orientation classification is unresolved");
}

void nest_shells(BRep& brep, double tolerance) {
    const int count = brep.shell_count();
    std::vector<std::vector<bool>> inside(count, std::vector<bool>(count, false));
    std::vector<int> depth(count, 0);
    for (int i = 0; i < count; ++i) {
        const Point point = shell_point(brep, i);
        for (int j = 0; j < count; ++j) {
            if (i == j)
                continue;

            const brep_classify::State state = brep_classify::point(shell_solid(brep, j), point, tolerance);
            require(state == brep_classify::State::Inside || state == brep_classify::State::Outside,
                "shell containment is ambiguous");
            inside[i][j] = state == brep_classify::State::Inside;
            depth[i] += inside[i][j] ? 1 : 0;
        }
    }
    for (int i = 0; i < count; ++i) {
        require(!inside[i][i], "cyclic shell containment");
        for (int j = i + 1; j < count; ++j)
            require(!inside[i][j] || !inside[j][i], "intersecting shell interiors");

        if (!outward(brep, i, tolerance))
            for (BRepRef& face : brep.m_shells[i].faces)
                face.orientation = brep_reverse(face.orientation);
    }
    brep.m_solids.clear();
    for (int i = 0; i < count; ++i) {
        if (depth[i] % 2 != 0)
            continue;

        std::vector<BRepRef> shells{{i, forward}};
        for (int j = 0; j < count; ++j)
            if (inside[j][i] && depth[j] == depth[i] + 1)
                shells.push_back({j, reversed});

        brep.add_solid(shells);
    }
}

}

ParameterError::ParameterError(
    const std::string& reason,
    const NurbsCurve& world,
    const NurbsCurve& uv,
    const NurbsSurface& support,
    double distance,
    curve_surface_intersection::ParameterMatch status
) : std::runtime_error(parameter_message(reason, world, uv, support, distance)),
    curve(world), pcurve(uv), surface(support), tolerance(distance), match(status) {}

BRep assemble(const std::vector<FaceUse>& faces, double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0)
        throw std::invalid_argument("BRep assembly requires a positive finite tolerance");
    if (faces.empty())
        return {};

    Assembly assembly{{}, {}, {}, {}, {}, tolerance};
    collect(assembly, faces);
    synchronize(assembly);
    partition(assembly);
    std::vector<BRepOrientation> orientations;
    for (const FaceUse& face : faces) {
        copy_face(assembly, face);
        BRepOrientation orientation = face.brep->face_orientation(face.face);
        if (face.reversed)
            orientation = brep_reverse(orientation);
        require(orientation == forward || orientation == reversed, "non-boundary face use");
        orientations.push_back(orientation);
    }
    require(assembly.result.is_valid(), "reconstructed face topology is invalid");
    build_shells(assembly.result, orientations);
    require(assembly.result.is_solid(), "reconstructed shells are not closed oriented solids");
    nest_shells(assembly.result, tolerance);
    require(assembly.result.is_solid(), "nested solid topology is invalid");
    return assembly.result;
}

}
