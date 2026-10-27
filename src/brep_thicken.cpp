#include "brep_thicken.h"
#include "brep_assemble.h"
#include "brep_offset.h"
#include "curve_surface_intersection.h"
#include "split.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>

namespace session_cpp::brep_thicken {
namespace {

constexpr BRepOrientation Forward = BRepOrientation::Forward;
constexpr BRepOrientation Reversed = BRepOrientation::Reversed;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

const NurbsCurve& world(const BRep& source, int edge) {
    return source.m_curves_3d[source.m_edges[edge].curve_3d_index];
}

NurbsCurve normalized(NurbsCurve curve) {
    require(curve.is_valid() && curve.dimension() == 3, "Face thickening requires valid three-dimensional edge curves");
    require(curve.set_domain(0, 1), "Face thickening could not normalize edge parameters");
    double scale = 0;
    for (int i = 0; i < curve.cv_count(); ++i) {
        const double weight = curve.weight(i);
        require(std::isfinite(weight) && weight > 0, "Face thickening requires positive finite rational edge weights");
        scale = std::max(scale, weight);
    }
    if (curve.is_rational())
        for (double& coordinate : curve.m_cv)
            coordinate /= scale;
    for (double coordinate : curve.m_cv)
        require(std::isfinite(coordinate), "Face thickening requires finite edge coordinates");
    return curve;
}

double choose(int degree, int index) {
    double value = 1;
    for (int i = 1; i <= index; ++i)
        value *= double(degree + 1 - i) / i;
    return value;
}

std::array<double, 4> homogeneous(const NurbsCurve& curve, int index) {
    const Point point = curve.get_cv(index);
    const double weight = curve.weight(index);
    return {point[0] * weight, point[1] * weight, point[2] * weight, weight};
}

std::array<std::array<double, 4>, 2> product_control(const NurbsCurve& a, const NurbsCurve& b, int index) {
    const int p = a.degree();
    const int q = b.degree();
    std::array<std::array<double, 4>, 2> result{};
    for (int i = std::max(0, index - q); i <= std::min(p, index); ++i) {
        const int j = index - i;
        const double coefficient = choose(p, i) * choose(q, j) / choose(p + q, index);
        const std::array<double, 4> first = homogeneous(a, i);
        const std::array<double, 4> second = homogeneous(b, j);
        for (int d = 0; d < 4; ++d) {
            result[0][d] += coefficient * first[d] * second[3];
            result[1][d] += coefficient * second[d] * first[3];
        }
    }
    result[1][3] = result[0][3];
    return result;
}

NurbsSurface ruled(const NurbsCurve& first, const NurbsCurve& second) {
    const NurbsCurve a = normalized(first);
    const NurbsCurve b = normalized(second);
    const int degree = a.degree() + b.degree();
    require(degree <= 32, "Face thickening ruled-wall degree exceeds the supported bound");
    std::vector<double> spans = a.get_span_vector();
    const std::vector<double> other = b.get_span_vector();
    spans.insert(spans.end(), other.begin(), other.end());
    std::sort(spans.begin(), spans.end());
    spans.erase(std::unique(spans.begin(), spans.end()), spans.end());
    require(spans.size() <= 1025, "Face thickening ruled-wall knot partition exceeds the supported bound");
    const int count = degree * (static_cast<int>(spans.size()) - 1) + 1;
    NurbsSurface result(3, true, degree + 1, 2, count, 2);
    result.m_nurbsknot[0].clear();
    result.m_nurbsknot[1] = {0, 1};
    for (double parameter : spans)
        result.m_nurbsknot[0].insert(result.m_nurbsknot[0].end(), degree, parameter);
    for (size_t span = 1; span < spans.size(); ++span) {
        NurbsCurve left = a;
        NurbsCurve right = b;
        require(left.trim(spans[span - 1], spans[span]) && right.trim(spans[span - 1], spans[span]),
            "Face thickening could not extract a ruled-wall Bezier interval");
        require(left.cv_count() == left.order() && right.cv_count() == right.order(),
            "Face thickening ruled-wall interval is not Bezier");
        for (int i = span == 1 ? 0 : 1; i <= degree; ++i) {
            const std::array<std::array<double, 4>, 2> controls = product_control(left, right, i);
            const int index = static_cast<int>(span - 1) * degree + i;
            for (int side = 0; side < 2; ++side) {
                require(controls[side][3] > 0 && std::isfinite(controls[side][3]),
                    "Face thickening ruled-wall weight is unresolved");
                std::copy(controls[side].begin(), controls[side].end(), result.cv(index, side));
            }
        }
    }
    require(result.is_valid(), "Face thickening could not construct a valid ruled wall");
    return result;
}

void certify(
    const NurbsCurve& curve,
    const NurbsCurve& pcurve,
    const NurbsSurface& surface,
    double tolerance
) {
    const curve_surface_intersection::ParameterMatch match = curve_surface_intersection::same_parameter(
        curve, pcurve, surface, tolerance
    );
    if (match != curve_surface_intersection::ParameterMatch::Matched)
        throw brep_assemble::ParameterError("Face thickening cannot certify boundary parameters", curve, pcurve, surface, tolerance, match);
}

struct Boundary {
    BRepRef use;
    int shifted;
    int count = 0;
};

std::map<int, Boundary> boundaries(const BRep& source, const BRep& shifted) {
    const BRepFace& first = source.m_faces[0];
    const BRepFace& second = shifted.m_faces[0];
    require(first.wires.size() == second.wires.size(), "Face thickening offset changed the trim-wire correspondence");
    std::map<int, Boundary> result;
    for (size_t i = 0; i < first.wires.size(); ++i) {
        const std::vector<BRepRef> a = source.wire_edges(first.wires[i]);
        const std::vector<BRepRef> b = shifted.wire_edges(second.wires[i]);
        require(a.size() == b.size(), "Face thickening offset changed the boundary-edge correspondence");
        for (size_t j = 0; j < a.size(); ++j) {
            const BRepRef use{a[j].index, brep_compose(source.face_orientation(0), a[j].orientation)};
            const std::map<int, Boundary>::iterator found = result.find(use.index);
            if (found == result.end())
                result[use.index] = {use, b[j].index, 1};
            else {
                require(found->second.shifted == b[j].index && found->second.use.orientation != use.orientation,
                    "Face thickening has inconsistent seam uses");
                ++found->second.count;
            }
            require(result[use.index].count <= 2, "Face thickening boundary is not manifold");
        }
    }
    return result;
}

std::array<NurbsCurve, 4> wall_curves(const NurbsCurve& source, const NurbsCurve& shifted) {
    const NurbsCurve a = normalized(source);
    const NurbsCurve b = normalized(shifted);
    return {a, NurbsCurve::create(false, 1, {a.point_at_end(), b.point_at_end()}),
        b, NurbsCurve::create(false, 1, {a.point_at_start(), b.point_at_start()})};
}

struct Parts {
    std::vector<BRep> shapes;
    std::vector<std::vector<split::EdgePiece>> pieces;
};

void wall(
    Parts& parts,
    const NurbsSurface& surface,
    const std::array<NurbsCurve, 4>& curves,
    const std::array<int, 4>& identities,
    BRepOrientation orientation,
    double tolerance
) {
    const std::array<Point, 4> uv = {Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0)};
    BRep result;
    result.add_surface(surface);
    for (const Point& point : uv)
        result.add_vertex(surface.point_at(point[0], point[1]), tolerance);
    std::vector<BRepRef> edges;
    std::vector<split::EdgePiece> pieces;
    for (int i = 0; i < 4; ++i) {
        const int a = i < 2 ? i : (i + 1) % 4;
        const int b = i < 2 ? i + 1 : i;
        const NurbsCurve pcurve = NurbsCurve::create(false, 1, {uv[a], uv[b]});
        certify(curves[i], pcurve, surface, tolerance * 0.5);
        const int edge = result.add_edge(result.add_curve_3d(curves[i]), a, b, tolerance);
        result.add_pcurve(edge, 0, result.add_curve_2d(pcurve));
        edges.push_back({edge, i < 2 ? Forward : Reversed});
        pieces.push_back({-1, identities[i], edge, 0, 1});
    }
    result.add_face(0, {{result.add_wire(edges), Forward}}, tolerance);
    result.add_shell({{0, orientation}});
    parts.shapes.push_back(std::move(result));
    parts.pieces.push_back(std::move(pieces));
}

void append_walls(
    Parts& parts,
    const std::map<int, Boundary>& edges,
    double thickness,
    double tolerance
) {
    const BRep source = parts.shapes[0];
    const BRep shifted = parts.shapes[1];
    int walls = 0;
    for (const std::pair<const int, Boundary>& entry : edges) {
        const int index = entry.first;
        const Boundary& boundary = entry.second;
        const BRepEdge& edge = source.m_edges[index];
        parts.pieces[0].push_back({-1, 2 * index, index, 0, 1});
        parts.pieces[1].push_back({-1, 2 * index + 1, boundary.shifted, 0, 1});
        if (boundary.count != 1 || edge.degenerated)
            continue;
        const std::array<NurbsCurve, 4> curves = wall_curves(world(source, index), world(shifted, boundary.shifted));
        const std::array<int, 4> identities = {2 * index, 2 * source.edge_count() + edge.end_vertex,
            2 * index + 1, 2 * source.edge_count() + edge.start_vertex};
        const BRepOrientation orientation = thickness > 0 ? boundary.use.orientation : brep_reverse(boundary.use.orientation);
        wall(parts, ruled(curves[0], curves[2]), curves, identities, orientation, tolerance);
        ++walls;
    }
    require(walls > 0, "Face thickening requires an open face with a nondegenerate free boundary");
}

}

BRep face(const BRep& source, double thickness, double tolerance) {
    if (!source.is_valid() || source.face_count() != 1 || source.solid_count() != 0)
        throw std::invalid_argument("Face thickening requires one valid trimmed open face");
    if (!std::isfinite(thickness) || !std::isfinite(tolerance) || tolerance <= 0 || std::abs(thickness) <= 2 * tolerance)
        throw std::invalid_argument("Face thickening requires finite thickness resolved above a positive finite tolerance");
    const BRep shifted = brep_offset::shape(source, thickness, tolerance * 0.125);
    Parts parts{{source, shifted}, {{}, {}}};
    append_walls(parts, boundaries(source, shifted), thickness, tolerance);
    std::vector<brep_assemble::FaceUse> selected;
    selected.push_back({&parts.shapes[0], 0, thickness > 0, &parts.pieces[0], -1});
    selected.push_back({&parts.shapes[1], 0, thickness < 0, &parts.pieces[1], -1});
    for (size_t i = 2; i < parts.shapes.size(); ++i)
        selected.push_back({&parts.shapes[i], 0, false, &parts.pieces[i], -1});
    BRep result = brep_assemble::assemble(selected, tolerance);
    require(result.is_solid() && result.solid_count() == 1 && result.shell_count() == 1,
        "Face thickening could not construct one closed oriented shell");
    result.name = source.name;
    result.surfacecolor = source.surfacecolor;
    result.is_visible = source.is_visible;
    result.is_locked = source.is_locked;
    result.width = source.width;
    return result;
}

}
