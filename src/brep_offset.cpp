#include "brep_offset.h"
#include "brep_offset_join.h"
#include "nurbs_offset.h"
#include "primitives.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

namespace session_cpp::brep_offset {
namespace {

constexpr BRepOrientation Forward = BRepOrientation::Forward;
constexpr BRepOrientation Reversed = BRepOrientation::Reversed;

std::vector<int> active_faces(const BRep& source) {
    std::vector<int> faces;
    if (!source.m_solids.empty()) {
        if (source.solid_count() != 1 || source.m_solids[0].shells.size() != 1)
            throw std::runtime_error("Native BRep offset requires shell selection for multiple solids or cavities");
        const BRepRef& shell = source.m_solids[0].shells[0];
        for (const BRepRef& face : source.m_shells[shell.index].faces)
            faces.push_back(face.index);
    } else if (!source.m_shells.empty()) {
        for (const BRepShell& shell : source.m_shells)
            for (const BRepRef& face : shell.faces)
                faces.push_back(face.index);
    } else {
        for (int i = 0; i < source.face_count(); ++i)
            faces.push_back(i);
    }
    std::vector<int> sorted = faces;
    std::sort(sorted.begin(), sorted.end());
    if (faces.empty() || std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
        throw std::runtime_error("Native BRep offset requires distinct active boundary faces");
    for (int face : faces) {
        const BRepOrientation orientation = source.face_orientation(face);
        if (orientation != Forward && orientation != Reversed)
            throw std::runtime_error("Native BRep offset requires an oriented boundary face");
    }
    return faces;
}

void check_focal_point(const NurbsSurface& source, double u, double v, double distance) {
    const std::vector<Vector> d = source.evaluate(u, v, 2);
    const Vector& su = d[3];
    const Vector& sv = d[1];
    const Vector area = su.cross(sv);
    const double length = area.magnitude();
    if (!(length > 0.0) || !std::isfinite(length))
        throw std::runtime_error("BRep offset has a singular interior surface point");
    const Vector n = area / length;
    const Vector au = d[5].cross(sv) + su.cross(d[4]);
    const Vector av = d[4].cross(sv) + su.cross(d[2]);
    const Vector nu = (au - n * n.dot(au)) / length;
    const Vector nv = (av - n * n.dot(av)) / length;
    const double e = su.dot(su);
    const double f = su.dot(sv);
    const double g = sv.dot(sv);
    const double a = su.dot(nu);
    const double b = (su.dot(nv) + sv.dot(nu)) * 0.5;
    const double c = sv.dot(nv);
    const double determinant = e * g - f * f;
    if (!(determinant > 0.0) || !std::isfinite(determinant))
        throw std::runtime_error("BRep offset has a singular or unresolved surface metric");
    const double trace = (g * a - 2.0 * f * b + e * c) / determinant;
    const double product = (a * c - b * b) / determinant;
    const double root = std::sqrt(std::max(0.0, trace * trace - 4.0 * product));
    const double first = 1.0 + distance * (trace - root) * 0.5;
    const double second = 1.0 + distance * (trace + root) * 0.5;
    if (!std::isfinite(first) || !std::isfinite(second) || first <= 1e-10 || second <= 1e-10)
        throw std::runtime_error("BRep offset reaches or crosses a focal collapse; topology reconstruction is required");
}

void check_focal_crossings(const NurbsSurface& source, double distance) {
    const std::vector<double> u = source.get_span_vector(0);
    const std::vector<double> v = source.get_span_vector(1);
    for (size_t i = 0; i + 1 < u.size(); ++i)
        for (size_t j = 0; j + 1 < v.size(); ++j)
            for (int a = 1; a < 4; ++a)
                for (int b = 1; b < 4; ++b)
                    check_focal_point(
                        source,
                        u[i] + (u[i + 1] - u[i]) * a / 4.0,
                        v[j] + (v[j + 1] - v[j]) * b / 4.0,
                        distance
                    );
}

struct TorusFrame {
    Point center;
    Vector x;
    Vector y;
    Vector z;
    double major;
    double minor;
};

bool torus_frame(const NurbsSurface& source, TorusFrame& frame) {
    if (!source.is_rational() || source.degree(0) != 2 || source.degree(1) != 2
        || source.cv_count(0) != 9 || source.cv_count(1) != 9)
        return false;
    const std::pair<double, double> u = source.domain(0);
    const std::pair<double, double> v = source.domain(1);
    const double du = u.second - u.first;
    const double dv = v.second - v.first;
    const Point outer = source.point_at(u.first, v.first);
    frame.center = outer + (source.point_at(u.first + du * 0.5, v.first) - outer) * 0.5;
    const double radius = outer.distance(frame.center);
    if (!(radius > 0.0))
        return false;
    frame.x = (outer - frame.center) / radius;
    frame.y = (source.point_at(u.first + du * 0.25, v.first) - frame.center) / radius;
    const double inside = (source.point_at(u.first, v.first + dv * 0.5) - frame.center).dot(frame.x);
    frame.major = (radius + inside) * 0.5;
    frame.minor = (radius - inside) * 0.5;
    if (!(frame.major > frame.minor && frame.minor > 0.0))
        return false;
    frame.z = (source.point_at(u.first, v.first + dv * 0.25) - frame.center - frame.x * frame.major) / frame.minor;
    return std::abs(frame.x.dot(frame.y)) < 1e-12 && std::abs(frame.x.dot(frame.z)) < 1e-12
        && std::abs(frame.y.dot(frame.z)) < 1e-12 && std::abs(frame.y.magnitude() - 1.0) < 1e-12
        && std::abs(frame.z.magnitude() - 1.0) < 1e-12;
}

NurbsSurface framed_torus(const TorusFrame& frame, double minor) {
    NurbsSurface result = Primitives::torus_surface(0, 0, 0, frame.major, minor);
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j) {
            const Point pole = result.get_cv(i, j);
            const Point point = frame.center + frame.x * pole[0] + frame.y * pole[1] + frame.z * pole[2];
            const double weight = result.weight(i, j);
            double* coordinates = result.cv(i, j);
            for (int d = 0; d < 3; ++d)
                coordinates[d] = point[d] * weight;
        }
    return result;
}

bool torus_offset(const NurbsSurface& source, double distance, NurbsSurface& result) {
    TorusFrame frame;
    if (!torus_frame(source, frame))
        return false;
    NurbsSurface reference = framed_torus(frame, frame.minor);
    for (int d = 0; d < 2; ++d) {
        const std::pair<double, double> domain = source.domain(d);
        if (!reference.set_domain(d, domain.first, domain.second))
            return false;
        if (reference.m_nurbsknot[d] != source.m_nurbsknot[d])
            return false;
    }
    const double scale = source.weight(0, 0);
    const double limit = 1024.0 * std::numeric_limits<double>::epsilon() * (frame.major + frame.minor);
    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j)
            if (source.get_cv(i, j).distance(reference.get_cv(i, j)) > limit
                || std::abs(source.weight(i, j) / scale - reference.weight(i, j)) > 1e-13)
                return false;
    const double orientation = frame.x.cross(frame.y).dot(frame.z) > 0.0 ? 1.0 : -1.0;
    const double minor = frame.minor + orientation * distance;
    if (minor <= limit || minor >= frame.major - limit)
        throw std::runtime_error("BRep torus offset collapses or self-intersects");
    result = framed_torus(frame, minor);
    result.m_nurbsknot[0] = source.m_nurbsknot[0];
    result.m_nurbsknot[1] = source.m_nurbsknot[1];
    return true;
}

Point lifted_point(const NurbsSurface& surface, const NurbsCurve& pcurve, double t) {
    const Point uv = pcurve.point_at(t);
    return surface.point_at(uv[0], uv[1]);
}

void validate_pcurve(const NurbsCurve& pcurve) {
    if (pcurve.dimension() < 2)
        throw std::invalid_argument("BRep offset requires two-dimensional trim coordinates");
    for (int i = 0; i < pcurve.cv_count(); ++i)
        if (!(pcurve.weight(i) > 0.0) || !std::isfinite(pcurve.weight(i)))
            throw std::invalid_argument("BRep offset requires positive finite trim weights");
}

bool iso_lift(const NurbsSurface& surface, const NurbsCurve& pcurve, NurbsCurve& result) {
    if (pcurve.degree() != 1 || pcurve.cv_count() != 2 || pcurve.weight(0) != pcurve.weight(1))
        return false;
    const Point a = pcurve.point_at_start();
    const Point b = pcurve.point_at_end();
    const int direction = a[0] == b[0] ? 1 : (a[1] == b[1] ? 0 : -1);
    if (direction < 0 || a[direction] == b[direction])
        return false;
    result = surface.iso_curve(direction, a[1 - direction]);
    if (!result.trim(std::min(a[direction], b[direction]), std::max(a[direction], b[direction])))
        return false;
    if (a[direction] > b[direction] && !result.reverse())
        throw std::runtime_error("BRep offset could not reverse its boundary curve");
    return result.set_domain(pcurve.domain_start(), pcurve.domain_end());
}

bool affine_lift(const NurbsSurface& surface, const NurbsCurve& pcurve, NurbsCurve& result) {
    if (surface.is_rational() || surface.cv_count(0) != 2 || surface.cv_count(1) != 2)
        return false;
    const Point origin = surface.get_cv(0, 0);
    const Vector x = surface.get_cv(1, 0) - origin;
    const Vector y = surface.get_cv(0, 1) - origin;
    const double roundoff = 128.0 * std::numeric_limits<double>::epsilon() * (x.magnitude() + y.magnitude());
    if (surface.get_cv(1, 1).distance(origin + x + y) > roundoff)
        return false;
    const std::pair<double, double> u = surface.domain(0);
    const std::pair<double, double> v = surface.domain(1);
    result = pcurve;
    if (result.dimension() != 3)
        return false;
    for (int i = 0; i < result.cv_count(); ++i) {
        const Point uv = pcurve.get_cv(i);
        const Point point = origin + x * ((uv[0] - u.first) / (u.second - u.first))
            + y * ((uv[1] - v.first) / (v.second - v.first));
        double* coordinates = result.cv(i);
        for (int d = 0; d < 3; ++d)
            coordinates[d] = point[d] * result.weight(i);
    }
    return true;
}

struct LiftValue {
    Point point;
    Vector derivative;
};

LiftValue lift_value(const NurbsSurface& surface, const NurbsCurve& pcurve, double t) {
    const std::vector<Vector> uv = pcurve.evaluate(t, 1);
    const std::vector<Vector> derivatives = surface.evaluate(uv[0][0], uv[0][1], 1);
    return {{derivatives[0][0], derivatives[0][1], derivatives[0][2]}, derivatives[2] * uv[1][0] + derivatives[1] * uv[1][1]};
}

Point bezier_point(const std::array<Point, 4>& controls, double t) {
    const double s = 1.0 - t;
    const double weights[] = {s * s * s, 3.0 * s * s * t, 3.0 * s * t * t, t * t * t};
    Point result;
    for (int d = 0; d < 3; ++d)
        for (int i = 0; i < 4; ++i)
            result[d] += weights[i] * controls[i][d];
    return result;
}

struct LiftBuilder {
    const NurbsSurface& surface;
    const NurbsCurve& pcurve;
    double tolerance;
    std::vector<Point> controls;
    std::vector<double> knots;

    void append(double a, double b, int depth) {
        if (controls.size() > 32768)
            throw std::runtime_error("BRep offset edge lift exceeds its control point limit");
        const LiftValue first = lift_value(surface, pcurve, std::nextafter(a, b));
        const LiftValue last = lift_value(surface, pcurve, std::nextafter(b, a));
        const Point pa = lifted_point(surface, pcurve, a);
        const Point pb = lifted_point(surface, pcurve, b);
        const std::array<Point, 4> poles = {pa, pa + first.derivative * ((b - a) / 3.0), pb - last.derivative * ((b - a) / 3.0), pb};
        double error = 0.0;
        for (int i = 1; i < 8; ++i) {
            const double fraction = i / 8.0;
            const double residual = lifted_point(surface, pcurve, a + (b - a) * fraction).distance(bezier_point(poles, fraction));
            if (!std::isfinite(residual))
                throw std::runtime_error("BRep offset edge lift has nonfinite geometry");
            error = std::max(error, residual);
        }
        if (error > tolerance * 0.25) {
            const double middle = (a + b) * 0.5;
            if (depth > 22 || controls.size() > 32768 || middle <= a || middle >= b)
                throw std::runtime_error("BRep offset edge lift cannot meet sampled tolerance");
            append(a, middle, depth + 1);
            append(middle, b, depth + 1);
            return;
        }
        if (controls.empty()) {
            controls.push_back(pa);
            knots.insert(knots.end(), 3, a);
        }
        for (int i = 1; i < 4; ++i)
            controls.push_back(poles[i]);
        knots.insert(knots.end(), 3, b);
    }
};

NurbsCurve lifted_curve(const NurbsSurface& surface, const NurbsCurve& pcurve, double tolerance) {
    NurbsCurve result;
    if (iso_lift(surface, pcurve, result) || affine_lift(surface, pcurve, result))
        return result;
    LiftBuilder builder{surface, pcurve, tolerance, {}, {}};
    const std::vector<double> spans = pcurve.get_span_vector();
    for (size_t i = 0; i + 1 < spans.size(); ++i)
        for (int j = 0; j < 4; ++j)
            builder.append(spans[i] + (spans[i + 1] - spans[i]) * j / 4.0,
                spans[i] + (spans[i + 1] - spans[i]) * (j + 1) / 4.0, 0);
    result = NurbsCurve(3, false, 4, static_cast<int>(builder.controls.size()));
    result.m_nurbsknot = builder.knots;
    for (int i = 0; i < result.cv_count(); ++i)
        result.set_cv(i, builder.controls[i]);
    if (!result.is_valid())
        throw std::runtime_error("BRep offset edge lift is invalid");
    return result;
}

struct FaceBuilder {
    const BRep& source;
    int face;
    BRep result;
    double tolerance;
    std::map<int, int> vertices;
    std::map<int, int> edges;

    int vertex(int source_vertex, const Point& point) {
        const std::map<int, int>::const_iterator found = vertices.find(source_vertex);
        if (found != vertices.end()) {
            if (result.m_vertices[found->second].point.distance(point) > tolerance)
                throw std::runtime_error("BRep offset trims disagree at a shared vertex");
            return found->second;
        }
        const int index = result.add_vertex(point, tolerance);
        vertices[source_vertex] = index;
        return index;
    }

    int edge(int source_edge) {
        const std::map<int, int>::const_iterator found = edges.find(source_edge);
        if (found != edges.end())
            return found->second;
        const BRepEdge& edge = source.m_edges[source_edge];
        const int first = source.pcurve_index(source_edge, face, Forward);
        const int second = source.pcurve_index(source_edge, face, Reversed);
        if (first < 0 || second < 0)
            throw std::runtime_error("BRep offset edge is missing its face pcurve");
        const NurbsCurve& pcurve = source.m_curves_2d[first];
        validate_pcurve(pcurve);
        const NurbsSurface& surface = result.m_surfaces[0];
        const Point a = lifted_point(surface, pcurve, pcurve.domain_start());
        const Point b = lifted_point(surface, pcurve, pcurve.domain_end());
        const int va = vertex(edge.start_vertex, a);
        const int vb = vertex(edge.end_vertex, b);
        const int world = edge.degenerated ? -1 : result.add_curve_3d(lifted_curve(surface, pcurve, tolerance));
        const int index = result.add_edge(world, va, vb, tolerance);
        verify_edge(index, pcurve);
        const int forward = result.add_curve_2d(pcurve);
        int reversed = -1;
        if (second != first) {
            const NurbsCurve& seam = source.m_curves_2d[second];
            validate_pcurve(seam);
            if (seam.domain() != pcurve.domain())
                throw std::runtime_error("BRep offset seam pcurves need the same edge parameter range");
            verify_edge(index, seam);
            reversed = result.add_curve_2d(seam);
        }
        result.add_pcurve(index, 0, forward, reversed);
        edges[source_edge] = index;
        return index;
    }

    void verify_edge(int index, const NurbsCurve& pcurve) const {
        const BRepEdge& edge = result.m_edges[index];
        for (int i = 0; i <= 97; ++i) {
            const double t = pcurve.domain_start() + (pcurve.domain_end() - pcurve.domain_start()) * i / 97.0;
            const Point lifted = lifted_point(result.m_surfaces[0], pcurve, t);
            const Point world = edge.degenerated ? result.m_vertices[edge.start_vertex].point
                : result.m_curves_3d[edge.curve_3d_index].point_at(t);
            const double residual = lifted.distance(world);
            if (!std::isfinite(residual) || residual > tolerance)
                throw std::runtime_error("BRep offset seam or edge cannot meet paired-curve tolerance");
        }
    }

    void topology() {
        std::vector<BRepRef> wires;
        for (const BRepRef& wire : source.m_faces[face].wires) {
            std::vector<BRepRef> uses;
            for (const BRepRef& use : source.m_wires[wire.index].edges)
                uses.push_back({edge(use.index), use.orientation});
            wires.push_back({result.add_wire(uses), wire.orientation});
        }
        result.add_face(0, wires, tolerance);
        result.m_faces[0].facecolor = source.m_faces[face].facecolor;
        if (!source.m_shells.empty() || !source.m_solids.empty()) {
            const int shell = result.add_shell({{0, source.face_orientation(face)}});
            if (!source.m_solids.empty())
                result.add_solid({{shell, Forward}});
        }
    }
};

OffsetFace offset_face(const BRep& source, int face, double distance, double tolerance) {
    const NurbsSurface surface = nurbs_offset::surface(
        source.m_surfaces[source.m_faces[face].surface_index], 0.0, tolerance * 0.5
    );
    const double signed_distance = source.face_orientation(face) == Reversed ? -distance : distance;
    if (!source.m_solids.empty())
        check_focal_crossings(surface, signed_distance);
    NurbsSurface shifted;
    if (!torus_offset(surface, signed_distance, shifted))
        shifted = nurbs_offset::surface(surface, signed_distance, tolerance * 0.5);
    FaceBuilder builder{source, face, {}, tolerance, {}, {}};
    builder.result.add_surface(shifted);
    builder.topology();
    return {std::move(builder.result), face, std::move(builder.edges)};
}

bool convex_planes(const BRep& source, double tolerance) {
    const std::vector<Plane> planes = source.face_planes();
    if (planes.size() != source.m_faces.size())
        return false;
    for (size_t face = 0; face < planes.size(); ++face) {
        const Plane& plane = planes[face];
        const NurbsSurface& surface = source.m_surfaces[source.m_faces[face].surface_index];
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j)
                if (std::abs(plane.z_axis().dot(surface.get_cv(i, j) - plane.origin())) > tolerance * 0.125)
                    return false;
        for (const BRepVertex& vertex : source.m_vertices)
            if (plane.z_axis().dot(vertex.point - plane.origin()) > tolerance * 0.125)
                return false;
    }
    for (const BRepEdge& edge : source.m_edges)
        if (edge.degenerated || !source.m_curves_3d[edge.curve_3d_index].is_linear(tolerance * 0.125))
            return false;
    return true;
}

}

BRep shape(const BRep& source, double distance, double tolerance) {
    if (!std::isfinite(distance) || !std::isfinite(tolerance) || tolerance <= 0.0 || !source.is_valid())
        throw std::invalid_argument("BRep offset requires valid geometry, finite distance and positive finite tolerance");
    if (distance == 0.0)
        return source;
    if (!source.m_solids.empty() && !source.is_solid())
        throw std::invalid_argument("BRep offset requires closed oriented input solid shells");
    const std::vector<int> active = active_faces(source);
    if (tolerance < 128.0 * std::numeric_limits<double>::epsilon() * std::abs(distance))
        throw std::runtime_error("BRep offset tolerance is below displacement resolution");
    std::vector<OffsetFace> faces;
    for (int face : active)
        faces.push_back(offset_face(source, face, distance, active.size() == 1 ? tolerance : tolerance * 0.125));
    BRep result;
    if (faces.size() == 1)
        result = std::move(faces[0].brep);
    else if (distance < 0.0 && source.solid_count() == 1 && convex_planes(source, tolerance)) {
        result = source.offset_convex(distance, tolerance);
        if (result.face_count() == 0)
            throw std::runtime_error("BRep offset erodes the convex solid to an empty or lower-dimensional result");
    } else {
        result = join(source, std::move(faces), distance, tolerance);
    }
    if (!result.is_valid() || (!source.m_solids.empty() && !result.is_solid()))
        throw std::runtime_error("BRep offset could not reconstruct valid closed trim topology");
    result.name = source.name;
    result.is_visible = source.is_visible;
    result.is_locked = source.is_locked;
    result.width = source.width;
    result.surfacecolor = source.surfacecolor;
    return result;
}

}
