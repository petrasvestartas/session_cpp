#include "brep_offset_join.h"
#include "brep_offset_inward.h"
#include "brep_assemble.h"
#include "curve_surface_intersection.h"
#include "split.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace session_cpp::brep_offset {
namespace {

constexpr BRepOrientation Forward = BRepOrientation::Forward;
constexpr BRepOrientation Reversed = BRepOrientation::Reversed;
using Field = std::function<Point(double, double)>;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

NurbsCurve normalized(NurbsCurve curve) {
    require(curve.set_domain(0.0, 1.0), "BRep offset could not synchronize boundary parameters");
    return curve;
}

const NurbsCurve& world(const BRep& source, int edge) {
    return source.m_curves_3d[source.m_edges[edge].curve_3d_index];
}

struct Use {
    int face;
    int edge;
    BRepOrientation orientation;
};

struct End {
    int vertex;
    int first;
    int second;
    NurbsCurve arc;
};

struct Parts {
    std::vector<BRep> shapes;
    std::vector<std::vector<split::EdgePiece>> pieces;
    std::vector<NurbsCurve> representatives;
};

int identity(Parts& parts, const NurbsCurve& curve) {
    const NurbsCurve key = normalized(curve);
    for (size_t i = 0; i < parts.representatives.size(); ++i) {
        const NurbsCurve& previous = parts.representatives[i];
        if (previous.is_rational() == key.is_rational() && previous.order() == key.order()
            && previous.m_cv == key.m_cv && previous.m_nurbsknot == key.m_nurbsknot)
            return static_cast<int>(i);
    }
    parts.representatives.push_back(key);
    return static_cast<int>(parts.representatives.size()) - 1;
}

void certify(const NurbsCurve& curve, const NurbsCurve& pcurve, const NurbsSurface& surface, double tolerance) {
    const curve_surface_intersection::ParameterMatch match = curve_surface_intersection::same_parameter(
        curve, pcurve, surface, tolerance
    );
    if (match != curve_surface_intersection::ParameterMatch::Matched)
        throw brep_assemble::ParameterError(
            "BRep offset cannot certify shared boundary parameters",
            curve,
            pcurve,
            surface,
            tolerance,
            match
        );
}

void certify_edge(const BRep& brep, int edge, const NurbsCurve& curve, double tolerance) {
    for (const BRepCurveOnSurface& pc : brep.m_edges[edge].pcurves)
        for (int index : {pc.curve_2d_index, pc.curve_2d_index_2})
            if (index >= 0)
                certify(curve, brep.m_curves_2d[index], brep.m_surfaces[pc.surface_index], tolerance);
}

Vector normal(const BRep& source, int face, int edge, BRepOrientation orientation, double t) {
    const int index = source.pcurve_index(edge, face, orientation);
    require(index >= 0, "BRep offset lacks an incident face pcurve");
    const NurbsCurve& pcurve = source.m_curves_2d[index];
    const Point uv = pcurve.point_at(pcurve.domain_start() + t * (pcurve.domain_end() - pcurve.domain_start()));
    const NurbsSurface& surface = source.m_surfaces[source.m_faces[face].surface_index];
    const std::vector<Vector> derivatives = surface.evaluate(uv[0], uv[1], 1);
    Vector result = derivatives[2].cross(derivatives[1]);
    require(result.normalize_self(), "BRep offset has an unresolved incident face normal");
    return source.face_orientation(face) == Reversed ? -result : result;
}

std::vector<std::vector<Use>> incidence(const BRep& source, const std::vector<OffsetFace>& faces) {
    std::vector<std::vector<Use>> result(source.edge_count());
    for (int i = 0; i < static_cast<int>(faces.size()); ++i)
        for (const BRepRef& wire : source.m_faces[faces[i].source].wires)
            for (const BRepRef& edge : source.wire_edges(wire))
                result[edge.index].push_back({i, faces[i].edges.at(edge.index), edge.orientation});
    for (int edge = 0; edge < source.edge_count(); ++edge)
        if (!result[edge].empty() && !source.m_edges[edge].degenerated)
            require(result[edge].size() == 2, "BRep offset requires a closed two-manifold source shell");
    std::vector<bool> reached(faces.size(), false);
    reached[0] = true;
    bool changed = true;
    while (changed) {
        changed = false;
        for (const std::vector<Use>& edge : result)
            if (edge.size() == 2 && reached[edge[0].face] != reached[edge[1].face]) {
                reached[edge[0].face] = reached[edge[1].face] = true;
                changed = true;
            }
    }
    require(std::all_of(reached.begin(), reached.end(), [](bool value) { return value; }),
        "BRep offset requires a connected source shell");
    return result;
}

bool smooth(const std::vector<OffsetFace>& faces, const Use& first, const Use& second, double tolerance) {
    const BRep& a = faces[first.face].brep;
    const BRep& b = faces[second.face].brep;
    const NurbsCurve& curve = world(a, first.edge);
    for (const BRepCurveOnSurface& pc : b.m_edges[second.edge].pcurves)
        for (int index : {pc.curve_2d_index, pc.curve_2d_index_2})
            if (index >= 0 && curve_surface_intersection::same_parameter(
                curve, b.m_curves_2d[index], b.m_surfaces[pc.surface_index], tolerance
            ) != curve_surface_intersection::ParameterMatch::Matched)
                return false;
    return true;
}

void check_convex(
    const BRep& source,
    const std::vector<OffsetFace>& faces,
    int edge,
    const Use& a,
    const Use& b,
    double distance,
    double tolerance
) {
    require(distance > 0.0, "BRep offset inward sharp joins require surface intersection cleanup");
    const NurbsCurve& spine = world(source, edge);
    const int first = faces[a.face].source;
    const int second = faces[b.face].source;
    const BRepOrientation orientation = brep_compose(source.face_orientation(first), a.orientation);
    const double sign = orientation == Forward ? 1.0 : -1.0;
    const double angular = std::min(0.01, tolerance / distance);
    for (int i = 1; i < 48; ++i) {
        const double t = i / 48.0;
        Vector tangent = spine.evaluate(spine.domain_start() + t * (spine.domain_end() - spine.domain_start()), 1)[1];
        require(tangent.normalize_self(), "BRep offset sharp edge has an undefined tangent");
        const Vector na = normal(source, first, edge, a.orientation, t);
        const Vector nb = normal(source, second, edge, b.orientation, t);
        require(sign * tangent.dot(na.cross(nb)) > angular,
            "BRep offset concave, tangent-transition or mixed sharp joins require intersection cleanup");
    }
}

std::array<Point, 3> arc_controls(const Point& center, const Point& first, const Point& last, double& weight) {
    const Vector a = first - center;
    const Vector b = last - center;
    const double ra = a.magnitude();
    const double rb = b.magnitude();
    require(ra > 0.0 && rb > 0.0, "BRep offset corner has a collapsed radial direction");
    const Vector na = a / ra;
    const Vector nb = b / rb;
    const double cosine = std::clamp(na.dot(nb), -1.0, 1.0);
    require(cosine > -1.0 + 1e-10, "BRep offset corner spans an unresolved half circle");
    weight = std::sqrt((1.0 + cosine) * 0.5);
    const Point middle = center + (na + nb) * ((ra + rb) * 0.5 / (1.0 + cosine));
    return {first, middle, last};
}

Point arc_point(const Point& center, const Point& first, const Point& last, double t) {
    double weight;
    const std::array<Point, 3> poles = arc_controls(center, first, last, weight);
    const double a = (1.0 - t) * (1.0 - t);
    const double b = 2.0 * t * (1.0 - t) * weight;
    const double c = t * t;
    Point result;
    for (int d = 0; d < 3; ++d)
        result[d] = (a * poles[0][d] + b * poles[1][d] + c * poles[2][d]) / (a + b + c);
    return result;
}

NurbsCurve arc(const Point& center, const Point& first, const Point& last) {
    double weight;
    const std::array<Point, 3> poles = arc_controls(center, first, last, weight);
    NurbsCurve result(3, true, 3, 3);
    result.m_nurbsknot = {0, 0, 1, 1};
    for (int i = 0; i < 3; ++i) {
        double* cv = result.cv(i);
        cv[3] = i == 1 ? weight : 1.0;
        for (int d = 0; d < 3; ++d)
            cv[d] = poles[i][d] * cv[3];
    }
    return result;
}

double choose(int n, int k) {
    double value = 1.0;
    for (int i = 1; i <= k; ++i)
        value *= double(n + 1 - i) / i;
    return value;
}

bool same_basis(const NurbsCurve& a, const NurbsCurve& b) {
    if (a.order() != b.order() || a.cv_count() != b.cv_count() || a.m_nurbsknot != b.m_nurbsknot)
        return false;
    for (int i = 0; i < a.cv_count(); ++i)
        if (a.weight(i) / a.weight(0) != b.weight(i) / b.weight(0))
            return false;
    return true;
}

bool radial_identity(
    const NurbsCurve& spine,
    const NurbsCurve& first,
    const NurbsCurve& second,
    double radius,
    double cosine
) {
    const int p = spine.degree();
    if (spine.cv_count() != p + 1 || p > 8)
        return false;
    std::vector<Vector> a, b, x;
    std::vector<double> w;
    const Point origin = spine.get_cv(0);
    for (int i = 0; i <= p; ++i) {
        const double weight = spine.weight(i) / spine.weight(0);
        w.push_back(weight);
        a.push_back((first.get_cv(i) - spine.get_cv(i)) * (weight / radius));
        b.push_back((second.get_cv(i) - spine.get_cv(i)) * (weight / radius));
        x.push_back((spine.get_cv(i) - origin) * weight);
    }
    const double epsilon = 4096.0 * std::numeric_limits<double>::epsilon();
    for (int k = 0; k <= 2 * p; ++k) {
        double aa = 0.0, bb = 0.0, ab = 0.0, scale = 0.0;
        for (int i = std::max(0, k - p); i <= std::min(p, k); ++i) {
            const int j = k - i;
            const double coefficient = choose(p, i) * choose(p, j) / choose(2 * p, k);
            aa += coefficient * (a[i].dot(a[j]) - w[i] * w[j]);
            bb += coefficient * (b[i].dot(b[j]) - w[i] * w[j]);
            ab += coefficient * (a[i].dot(b[j]) - cosine * w[i] * w[j]);
            scale += coefficient * (a[i].magnitude() * a[j].magnitude() + b[i].magnitude() * b[j].magnitude() + w[i] * w[j]);
        }
        if (std::max({std::abs(aa), std::abs(bb), std::abs(ab)}) > epsilon * scale)
            return false;
    }
    std::vector<Vector> tangent(2 * p);
    double scale = 0.0;
    for (int k = 0; k < 2 * p; ++k)
        for (int i = std::max(0, k - p); i <= std::min(p - 1, k); ++i) {
            const int j = k - i;
            const double coefficient = choose(p - 1, i) * choose(p, j) / choose(2 * p - 1, k);
            const Vector value = ((x[i + 1] - x[i]) * w[j] - x[j] * (w[i + 1] - w[i])) * p;
            tangent[k] += value * coefficient;
            scale = std::max(scale, value.magnitude());
        }
    for (int k = 0; k < 3 * p; ++k) {
        double first_dot = 0.0, second_dot = 0.0;
        for (int i = std::max(0, k - 2 * p + 1); i <= std::min(p, k); ++i) {
            const double coefficient = choose(p, i) * choose(2 * p - 1, k - i) / choose(3 * p - 1, k);
            first_dot += coefficient * a[i].dot(tangent[k - i]);
            second_dot += coefficient * b[i].dot(tangent[k - i]);
        }
        if (std::max(std::abs(first_dot), std::abs(second_dot)) > epsilon * scale)
            return false;
    }
    return true;
}

bool exact_tube(const NurbsCurve& spine, const NurbsCurve& first, const NurbsCurve& second, NurbsSurface& result) {
    const bool line = spine.degree() == 1 && spine.cv_count() == 2;
    if ((!line && (!spine.is_closed() || spine.degree() != 2)) || !same_basis(spine, first) || !same_basis(spine, second))
        return false;
    const Vector a = first.point_at_start() - spine.point_at_start();
    const Vector b = second.point_at_start() - spine.point_at_start();
    const double radius = a.magnitude();
    if (!(radius > 0.0))
        return false;
    const double cosine = a.dot(b) / (radius * radius);
    if (!(cosine > -1.0 + 1e-10 && cosine < 1.0 - 1e-10))
        return false;
    const std::vector<double> knots = spine.get_span_vector();
    for (size_t i = 1; i < knots.size(); ++i) {
        NurbsCurve c = spine, x = first, y = second;
        if (!c.trim(knots[i - 1], knots[i]) || !x.trim(knots[i - 1], knots[i]) || !y.trim(knots[i - 1], knots[i])
            || !radial_identity(c, x, y, radius, cosine))
            return false;
    }
    const double weight = std::sqrt((1.0 + cosine) * 0.5);
    result = NurbsSurface(3, true, spine.order(), 3, spine.cv_count(), 3);
    result.m_nurbsknot[0] = spine.m_nurbsknot;
    result.m_nurbsknot[1] = {0, 0, 1, 1};
    for (int i = 0; i < spine.cv_count(); ++i) {
        const Point center = spine.get_cv(i);
        const Point middle = center + ((first.get_cv(i) - center) + (second.get_cv(i) - center)) / (1.0 + cosine);
        const std::array<Point, 3> poles = {first.get_cv(i), middle, second.get_cv(i)};
        for (int j = 0; j < 3; ++j) {
            double* cv = result.cv(i, j);
            cv[3] = spine.weight(i) / spine.weight(0) * (j == 1 ? weight : 1.0);
            for (int d = 0; d < 3; ++d)
                cv[d] = poles[j][d] * cv[3];
        }
    }
    return true;
}

std::vector<double> parameters(int count) {
    std::vector<double> result;
    for (int i = 0; i < count; ++i)
        result.push_back(double(i) / (count - 1));
    return result;
}

NurbsSurface interpolate(const Field& field, const std::vector<double>& u, const std::vector<double>& v) {
    std::vector<NurbsCurve> rows;
    for (double x : u) {
        std::vector<Point> points;
        for (double y : v)
            points.push_back(field(x, y));
        rows.push_back(NurbsCurve::create_interpolated(points, v));
        require(rows.back().is_valid(), "BRep offset join interpolation failed");
    }
    std::vector<NurbsCurve> columns;
    for (int j = 0; j < rows[0].cv_count(); ++j) {
        std::vector<Point> points;
        for (const NurbsCurve& row : rows)
            points.push_back(row.get_cv(j));
        columns.push_back(NurbsCurve::create_interpolated(points, u));
        require(columns.back().is_valid(), "BRep offset join interpolation failed");
    }
    NurbsSurface result(3, false, 4, 4, columns[0].cv_count(), rows[0].cv_count());
    result.m_nurbsknot[0] = columns[0].m_nurbsknot;
    result.m_nurbsknot[1] = rows[0].m_nurbsknot;
    for (int i = 0; i < result.cv_count(0); ++i)
        for (int j = 0; j < result.cv_count(1); ++j)
            result.set_cv(i, j, columns[j].get_cv(i));
    return result;
}

NurbsSurface approximate(const Field& field, double tolerance) {
    int nu = 5;
    int nv = 5;
    while (nu <= 129 && nv <= 129) {
        const std::vector<double> u = parameters(nu);
        const std::vector<double> v = parameters(nv);
        NurbsSurface result = interpolate(field, u, v);
        bool refine_u = false;
        bool refine_v = false;
        for (int i = 0; i < nu - 1; ++i)
            for (int j = 0; j < nv - 1; ++j)
                for (int a = 0; a <= 4; ++a)
                    for (int b = 0; b <= 4; ++b) {
                        const double x = (i + a / 4.0) / (nu - 1);
                        const double y = (j + b / 4.0) / (nv - 1);
                        if (field(x, y).distance(result.point_at(x, y)) <= tolerance * 0.0625)
                            continue;
                        require(a % 4 != 0 || b % 4 != 0, "BRep offset join interpolation exceeds tolerance at a grid node");
                        refine_u = refine_u || a % 4 != 0;
                        refine_v = refine_v || b % 4 != 0;
                    }
        if (!refine_u && !refine_v)
            return result;
        nu = refine_u ? nu * 2 - 1 : nu;
        nv = refine_v ? nv * 2 - 1 : nv;
    }
    throw std::runtime_error("BRep offset join cannot meet sampled surface tolerance");
}

NurbsCurve pcurve(int side) {
    const std::array<Point, 4> first = {Point(0, 0, 0), Point(1, 0, 0), Point(0, 1, 0), Point(0, 0, 0)};
    const std::array<Point, 4> last = {Point(1, 0, 0), Point(1, 1, 0), Point(1, 1, 0), Point(0, 1, 0)};
    return NurbsCurve::create(false, 1, {first[side], last[side]});
}

int vertex(BRep& brep, const Point& point, double tolerance) {
    for (int i = 0; i < brep.vertex_count(); ++i)
        if (brep.m_vertices[i].point.distance(point) <= tolerance)
            return i;
    return brep.add_vertex(point, tolerance);
}

void append_patch(
    Parts& parts,
    const NurbsSurface& surface,
    const std::array<NurbsCurve, 4>& curves,
    const Point& center,
    double tolerance,
    bool pole
) {
    BRep result;
    result.add_surface(surface);
    std::vector<BRepRef> edges;
    std::vector<split::EdgePiece> pieces;
    for (int i = 0; i < 4; ++i) {
        const NurbsCurve pc = pcurve(i);
        certify(curves[i], pc, surface, tolerance * 0.5);
        const int a = vertex(result, curves[i].point_at_start(), tolerance);
        const int b = pole && i == 0 ? a : vertex(result, curves[i].point_at_end(), tolerance);
        const int edge = result.add_edge(pole && i == 0 ? -1 : result.add_curve_3d(curves[i]), a, b, tolerance);
        result.add_pcurve(edge, 0, result.add_curve_2d(pc));
        edges.push_back({edge, i < 2 ? Forward : Reversed});
        pieces.push_back({-1, identity(parts, curves[i]), edge, 0, 1});
    }
    result.add_face(0, {{result.add_wire(edges), Forward}}, tolerance);
    const Point middle = surface.point_at(0.5, 0.5);
    const BRepOrientation orientation = surface.normal_at(0.5, 0.5).dot(middle - center) > 0.0 ? Forward : Reversed;
    result.add_shell({{0, orientation}});
    parts.shapes.push_back(std::move(result));
    parts.pieces.push_back(std::move(pieces));
}

void tube(
    Parts& parts,
    std::vector<End>& ends,
    const BRep& source,
    const std::vector<OffsetFace>& faces,
    int edge,
    const Use& first,
    const Use& second,
    double tolerance
) {
    const NurbsCurve spine = normalized(world(source, edge));
    const NurbsCurve a = normalized(world(faces[first.face].brep, first.edge));
    const NurbsCurve b = normalized(world(faces[second.face].brep, second.edge));
    const Field field = [&](double u, double v) { return arc_point(spine.point_at(u), a.point_at(u), b.point_at(u), v); };
    const NurbsCurve start = arc(spine.point_at_start(), a.point_at_start(), b.point_at_start());
    const NurbsCurve finish = arc(spine.point_at_end(), a.point_at_end(), b.point_at_end());
    NurbsSurface surface;
    if (!exact_tube(spine, a, b, surface))
        surface = approximate(field, tolerance);
    append_patch(parts, surface, {a, finish, b, start}, spine.point_at_middle(), tolerance, false);
    const BRepEdge& original = source.m_edges[edge];
    ends.push_back({original.start_vertex, first.face, second.face, start});
    ends.push_back({original.end_vertex, first.face, second.face, finish});
}

void check_corner(const std::vector<const End*>& borders, const std::vector<Point>& points, double tolerance) {
    std::vector<std::vector<int>> adjacent(points.size());
    for (const End* border : borders) {
        const Point a = border->arc.point_at_start();
        const Point b = border->arc.point_at_end();
        const int first = static_cast<int>(std::find_if(points.begin(), points.end(), [&](const Point& p) { return p.distance(a) <= tolerance; }) - points.begin());
        const int second = static_cast<int>(std::find_if(points.begin(), points.end(), [&](const Point& p) { return p.distance(b) <= tolerance; }) - points.begin());
        require(first < static_cast<int>(points.size()) && second < static_cast<int>(points.size()) && first != second,
            "BRep offset corner has a collapsed or unmatched tube boundary");
        adjacent[first].push_back(second);
        adjacent[second].push_back(first);
    }
    std::vector<bool> reached(points.size(), false);
    int current = 0;
    int previous = -1;
    for (size_t i = 0; i < points.size(); ++i) {
        require(!reached[current] && adjacent[current].size() == 2, "BRep offset corner does not have one cyclic tube neighborhood");
        reached[current] = true;
        const int next = adjacent[current][0] == previous ? adjacent[current][1] : adjacent[current][0];
        previous = current;
        current = next;
    }
    require(current == 0, "BRep offset corner has an open tube neighborhood");
}

void corners(Parts& parts, const BRep& source, const std::vector<End>& ends, double distance, double tolerance) {
    for (int vertex = 0; vertex < source.vertex_count(); ++vertex) {
        std::vector<const End*> borders;
        std::vector<Point> points;
        for (const End& end : ends)
            if (end.vertex == vertex) {
                borders.push_back(&end);
                for (const Point point : {end.arc.point_at_start(), end.arc.point_at_end()})
                    if (std::none_of(points.begin(), points.end(), [&](const Point& p) { return p.distance(point) <= tolerance; }))
                        points.push_back(point);
            }
        if (points.empty())
            continue;
        check_corner(borders, points, tolerance);
        if (points.size() < 3)
            continue;
        require(borders.size() == points.size(), "BRep offset vertex requires mixed-join intersection cleanup");
        const Point center = source.m_vertices[vertex].point;
        Vector normal;
        for (const Point& point : points)
            normal += point - center;
        require(normal.normalize_self(), "BRep offset corner has no resolved interior radial direction");
        const Point pole = center + normal * distance;
        for (const End* border : borders) {
            const NurbsCurve& base = border->arc;
            const Field field = [&](double u, double v) { return arc_point(center, pole, base.point_at(u), v); };
            const NurbsCurve left = arc(center, pole, base.point_at_start());
            const NurbsCurve right = arc(center, pole, base.point_at_end());
            const NurbsCurve collapsed = NurbsCurve::create(false, 1, {pole, pole});
            append_patch(parts, approximate(field, tolerance), {collapsed, right, base, left}, center, tolerance, true);
        }
    }
}

void share_smooth(std::vector<OffsetFace>& faces, const Use& first, const Use& second, double tolerance) {
    BRep& target = faces[second.face].brep;
    NurbsCurve curve = world(faces[first.face].brep, first.edge);
    const NurbsCurve& previous = world(target, second.edge);
    require(curve.set_domain(previous.domain_start(), previous.domain_end()), "BRep offset could not share a smooth boundary");
    certify_edge(target, second.edge, curve, tolerance * 0.5);
    target.m_curves_3d[target.m_edges[second.edge].curve_3d_index] = curve;
}

}

OffsetFace inward_tube(const BRep& source, const std::vector<OffsetFace>& faces,
    int edge, int first, int second, double tolerance) {
    Parts parts;
    std::vector<End> ends;
    tube(parts, ends, source, faces, edge,
        {first, faces[first].edges.at(edge), Forward},
        {second, faces[second].edges.at(edge), Forward}, tolerance);
    BRep result = std::move(parts.shapes[0]);
    result.m_shells[0].faces[0].orientation = brep_reverse(result.m_shells[0].faces[0].orientation);
    require(result.is_valid(), "Inward tube has invalid boundary geometry");
    return {std::move(result), -1, {}};
}

BRep join(
    const BRep& source,
    std::vector<OffsetFace> faces,
    double distance,
    double tolerance
) {
    require(source.solid_count() == 1, "Native multi-face offset currently requires one closed solid");
    const std::vector<std::vector<Use>> edges = incidence(source, faces);
    if (distance < 0.0)
        for (const std::vector<Use>& edge : edges)
            if (edge.size() == 2 && edge[0].face != edge[1].face
                && !smooth(faces, edge[0], edge[1], tolerance * 0.25))
                return inward(source, std::move(faces), distance, tolerance);
    Parts parts;
    std::vector<End> ends;
    for (int edge = 0; edge < source.edge_count(); ++edge) {
        if (edges[edge].empty() || source.m_edges[edge].degenerated)
            continue;
        const Use& a = edges[edge][0];
        const Use& b = edges[edge][1];
        if (a.face == b.face)
            continue;
        certify_edge(source, edge, world(source, edge), tolerance * 0.5);
        if (smooth(faces, a, b, tolerance * 0.25)) {
            share_smooth(faces, a, b, tolerance);
            continue;
        }
        check_convex(source, faces, edge, a, b, distance, tolerance);
        tube(parts, ends, source, faces, edge, a, b, tolerance);
    }
    corners(parts, source, ends, distance, tolerance);
    for (OffsetFace& face : faces) {
        std::vector<split::EdgePiece> pieces;
        for (const std::pair<const int, int>& edge : face.edges)
            if (!face.brep.m_edges[edge.second].degenerated)
                pieces.push_back({-1, identity(parts, world(face.brep, edge.second)), edge.second, 0, 1});
        parts.shapes.push_back(std::move(face.brep));
        parts.pieces.push_back(std::move(pieces));
    }
    std::vector<brep_assemble::FaceUse> selected;
    for (size_t i = 0; i < parts.shapes.size(); ++i)
        selected.push_back({&parts.shapes[i], 0, false, &parts.pieces[i], -1});
    return brep_assemble::assemble(selected, tolerance);
}

}
