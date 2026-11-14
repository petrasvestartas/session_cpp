#include "brep_classify.h"
#include "brep_face_boundary.h"
#include "closest.h"
#include "surface_ray.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>

namespace session_cpp::brep_classify {
namespace {

struct CurvePart {
    NurbsCurve curve;
    int depth = 0;
};

struct RayFrame {
    Point origin;
    Vector direction;
    double tolerance;
};

void check_tolerance(double tolerance) {
    if (!std::isfinite(tolerance) || tolerance <= 0.0)
        throw std::invalid_argument("BRep classification requires a finite positive tolerance");
}

bool finite(const xyz& p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

double segment_distance(const xyz& p, const xyz& a, const xyz& b) {
    const xyz delta = b - a;
    const double squared = delta.dot(delta);
    const double t = squared > 0.0 ? std::clamp((p - a).dot(delta) / squared, 0.0, 1.0) : 0.0;
    return p.distance(a + delta * t);
}

int crossing(const xyz& a, const xyz& b, const xyz& query) {
    if ((a[1] > query[1]) == (b[1] > query[1]))
        return 0;
    const double t = (query[1] - a[1]) / (b[1] - a[1]);
    if (a[0] + t * (b[0] - a[0]) <= query[0])
        return 0;
    return b[1] > a[1] ? 1 : -1;
}

/// The control point NurbsCurve::get_cv returns.
xyz control(const NurbsCurve& curve, int index) {
    const double* cv = curve.cv(index);
    const int dim = curve.dimension();
    const double weight = curve.is_rational() ? cv[dim] : 1.0;
    if (weight == 0.0)
        return {0.0, 0.0, 0.0};
    return {cv[0] / weight, dim > 1 ? cv[1] / weight : 0.0, dim > 2 ? cv[2] / weight : 0.0};
}

/// The box of the control points of a piece with ends a and b and their largest distance from its chord; false where a control is not finite with a positive weight.
bool chord_hull(const NurbsCurve& curve, const xyz& a, const xyz& b, xyz& lo, xyz& hi, double& error) {
    lo = hi = a;
    error = 0.0;
    for (int i = 0; i < curve.cv_count(); ++i) {
        const xyz p = control(curve, i);
        if (!finite(p) || !std::isfinite(curve.weight(i)) || !(curve.weight(i) > 0.0))
            return false;
        for (int d = 0; d < 2; ++d) {
            lo[d] = std::min(lo[d], p[d]);
            hi[d] = std::max(hi[d], p[d]);
        }
        error = std::max(error, segment_distance(p, a, b));
    }
    return true;
}

/// checked: the owner of the curve is known to be valid.
State curve_winding(const NurbsCurve& curve, const xyz& query, double height, double tolerance, int& winding, bool checked) {
    const xyz ray{query[0], height, 0.0};
    xyz lo, hi;
    double error;
    // A straight piece is the first pass of the loop below, taken on the curve itself.
    if (curve.order() == 2 && curve.cv_count() == 2) {
        // Without weights the ends of a valid straight piece are its two control points to the bit.
        const bool plain = !curve.is_rational() && (checked || curve.is_valid());
        const xyz a = plain ? control(curve, 0) : curve.point(curve.domain_start());
        const xyz b = plain ? control(curve, 1) : curve.point(curve.domain_end());
        if (!chord_hull(curve, a, b, lo, hi, error))
            return State::Unknown;
        if (hi[0] < query[0] - tolerance || hi[1] < query[1] - tolerance || lo[1] > query[1] + tolerance)
            return State::Outside;
        if (lo[0] <= query[0] + tolerance && error <= tolerance * 0.125 && segment_distance(query, a, b) <= tolerance + error)
            return State::Boundary;
        if (lo[0] > query[0] + tolerance || error <= tolerance * 0.125) {
            winding += crossing(a, b, ray);
            return State::Outside;
        }
    }
    // The curve itself is the first part; only its pieces are held.
    std::vector<CurvePart> work;
    CurvePart held;
    const NurbsCurve* part = &curve;
    int depth = 0;
    for (size_t iteration = 0; iteration < 100000; ++iteration) {
        const xyz a = part->point(part->domain_start());
        const xyz b = part->point(part->domain_end());
        if (!chord_hull(*part, a, b, lo, hi, error))
            return State::Unknown;
        if (hi[0] < query[0] - tolerance || hi[1] < query[1] - tolerance || lo[1] > query[1] + tolerance) {
        } else if (lo[0] > query[0] + tolerance) {
            winding += crossing(a, b, ray);
        } else if (error <= tolerance * 0.125) {
            if (segment_distance(query, a, b) <= tolerance + error)
                return State::Boundary;
            winding += crossing(a, b, ray);
        } else {
            const double middle = std::midpoint(part->domain_start(), part->domain_end());
            NurbsCurve left;
            NurbsCurve right;
            if (depth >= 64 || !part->split(middle, left, right))
                return State::Unknown;
            work.push_back({std::move(right), depth + 1});
            work.push_back({std::move(left), depth + 1});
        }
        if (work.empty())
            return State::Outside;
        held = std::move(work.back());
        work.pop_back();
        part = &held.curve;
        depth = held.depth;
    }
    return State::Unknown;
}

/// The v of both ends of a pcurve; checked: its owner is known to be valid.
std::array<double, 2> end_heights(const NurbsCurve& curve, bool checked) {
    // The ends of a valid straight piece without weights are its two control points to the bit.
    const bool plain = curve.order() == 2 && curve.cv_count() == 2 && !curve.is_rational() &&
        curve.dimension() > 1 && (checked || curve.is_valid());
    return {plain ? curve.cv(0)[1] : curve.point(curve.domain_start())[1],
        plain ? curve.cv(1)[1] : curve.point(curve.domain_end())[1]};
}

double clear_height(const std::vector<std::array<double, 2>>& ends, double height, double tolerance) {
    const auto clear = [&ends](double candidate) {
        const double margin = 64.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(candidate));
        for (const std::array<double, 2>& end : ends)
            if (std::abs(end[0] - candidate) <= margin || std::abs(end[1] - candidate) <= margin)
                return false;
        return true;
    };
    for (double share : {0.0, 0.25, -0.25, 0.125, -0.125, 0.1875, -0.1875, 0.0625, -0.0625})
        if (clear(height + share * tolerance))
            return height + share * tolerance;
    return height;
}

std::vector<std::array<double, 2>> wire_ends(const BRep& brep, const brep_face_boundary::Wire& wire, bool checked) {
    std::vector<std::array<double, 2>> ends;
    for (const auto& use : wire.uses)
        if (use.pcurve >= 0 && use.pcurve < static_cast<int>(brep.m_curves_2d.size()))
            ends.push_back(end_heights(brep.m_curves_2d[use.pcurve], checked));
    return ends;
}

/// known: the ends of the wire's pcurves, of a valid owner; without them they are found here.
State wire_state(const BRep& brep, const brep_face_boundary::Wire& wire, const std::vector<std::array<double, 2>>* known,
    const Point& uv, double tolerance) {
    std::vector<std::array<double, 2>> found;
    if (!known)
        found = wire_ends(brep, wire, false);
    const double height = clear_height(known ? *known : found, uv[1], tolerance);
    int winding = 0;
    for (const auto& use : wire.uses) {
        const auto& edge = use.edge;
        if (edge.orientation == BRepOrientation::Internal || edge.orientation == BRepOrientation::External)
            continue;
        const int ci = use.pcurve;
        if (ci < 0 || ci >= static_cast<int>(brep.m_curves_2d.size()))
            return State::Unknown;
        int contribution = 0;
        const State state = curve_winding(brep.m_curves_2d[ci], uv, height, tolerance, contribution, known != nullptr);
        if (state == State::Boundary || state == State::Unknown)
            return state;
        winding += edge.orientation == BRepOrientation::Reversed ? -contribution : contribution;
    }
    return winding == 0 ? State::Outside : State::Inside;
}

/// The boundary of a face of a valid model and the ends of its pcurves.
void lay_out(const BRep& brep, int index, Face& face) {
    face.boundary = brep_face_boundary::extract(brep, index);
    for (size_t wire = 0; face.boundary.complete && wire < face.boundary.wires.size(); ++wire)
        face.ends.push_back(wire_ends(brep, face.boundary.wires[wire], true));
}

State face_state(const brep_face_boundary::Result& boundary, const std::vector<std::vector<std::array<double, 2>>>* ends,
    const Point& uv, double tolerance) {
    check_tolerance(tolerance);
    if (!finite(uv))
        throw std::invalid_argument("BRep classification requires a finite UV point");
    if (!boundary.complete || !boundary.owner || boundary.wires.empty() ||
        boundary.face < 0 || boundary.face >= boundary.owner->face_count())
        return State::Unknown;
    const BRep& brep = *boundary.owner;
    const State outer = wire_state(brep, boundary.wires[0], ends ? &(*ends)[0] : nullptr, uv, tolerance);
    if (outer != State::Inside)
        return outer;
    for (size_t i = 1; i < boundary.wires.size(); ++i) {
        const State hole = wire_state(brep, boundary.wires[i], ends ? &(*ends)[i] : nullptr, uv, tolerance);
        if (hole == State::Unknown || hole == State::Boundary)
            return hole;
        if (hole == State::Inside)
            return State::Outside;
    }
    return State::Inside;
}

bool append_hit(RayResult& result, const Solid& solid, int face,
    const surface_ray::Root& root, const RayFrame& frame) {
    const BRep& brep = solid.brep;
    std::array<xyz, 6> values;
    if (!brep.m_surfaces[brep.m_faces[face].surface_index].jet(root.u, root.v, 1, values))
        return false;
    const double speed = std::max(values[1].length(), values[2].length());
    if (!std::isfinite(speed) || !(speed > 0.0))
        return false;
    const double infinity = std::numeric_limits<double>::infinity();
    const double du = std::nextafter(std::max(std::abs(root.u - root.enclosure[0]),
        std::abs(root.u - root.enclosure[1])), infinity);
    const double dv = std::nextafter(std::max(std::abs(root.v - root.enclosure[2]),
        std::abs(root.v - root.enclosure[3])), infinity);
    const double radius = std::nextafter(std::hypot(du, dv), infinity);
    const double tolerance = std::nextafter(frame.tolerance / speed + radius, infinity);
    if (!std::isfinite(tolerance))
        return false;
    const State trim = face_uv(solid, face, {root.u, root.v, 0.0}, tolerance);
    if (trim == State::Unknown)
        return false;
    if (trim == State::Outside)
        return true;
    const int crossing = solid.faces[face].orientation == BRepOrientation::Reversed ? -root.crossing : root.crossing;
    result.hits.push_back({root.point, root.distance, root.u, root.v, face, crossing, trim, root.distance_enclosure});
    return true;
}

// Roots of one ray on the surfaces it has met, by surface index and advance: faces split from one face share their surface.
using SurfaceRoots = std::map<std::pair<int, double>, surface_ray::Result>;

void intersect_face(RayResult& result, const Solid& solid, int face, const RayFrame& frame, SurfaceRoots& met,
    const Vector* grazed = nullptr) {
    const BRep& brep = solid.brep;
    const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
    surface_ray::Options options;
    options.include_origin = true;
    // The origin lies on this face's surface outside its trim: that root is no hit, and at distance zero it can not be isolated. The ray starts where it has left the surface, unless it runs too flat along it to say where that is.
    double advance = 0.0;
    if (grazed) {
        Vector unit = frame.direction;
        const double across = unit.normalize_self() ? std::abs(unit.dot(*grazed)) : 0.0;
        if (across < 0.1) {
            result.complete = false;
            result.failure = "Unresolved ray along the surface of face " + std::to_string(face);
            result.unresolved_face = face;
            return;
        }
        advance = 64.0 * frame.tolerance / across;
    }
    const auto [found, fresh] = met.try_emplace({brep.m_faces[face].surface_index, advance});
    surface_ray::Result& intersections = found->second;
    if (fresh) {
        Vector step = frame.direction;
        step.normalize_self();
        intersections = surface_ray::intersect(surface, advance > 0.0 ? frame.origin + step * advance : frame.origin,
            frame.direction, frame.tolerance, options);
        for (auto& root : intersections.roots) {
            root.distance += advance;
            root.distance_enclosure[0] += advance;
            root.distance_enclosure[1] += advance;
        }
    }
    if (!intersections.complete) {
        result.complete = false;
        result.failure = "Unresolved ray on face " + std::to_string(face) + ": " + intersections.failure;
        result.unresolved_face = face;
        result.unresolved_domain = {surface.domain(0).first, surface.domain(0).second,
            surface.domain(1).first, surface.domain(1).second};
    }
    for (const auto& root : intersections.roots)
        if (!append_hit(result, solid, face, root, frame)) {
            result.complete = false;
            result.failure = "Unresolved ray trim on face " + std::to_string(face);
            result.unresolved_face = face;
            result.unresolved_domain = root.enclosure;
        }
}

/// The ray on a checked model; grazed: faces on whose surfaces the origin lies outside their trims, with their normals there.
RayResult cast(const Solid& solid, const RayFrame& frame, const std::vector<std::pair<int, Vector>>& grazed = {}) {
    RayResult result;
    SurfaceRoots met;
    for (int face = 0; face < static_cast<int>(solid.faces.size()); ++face) {
        if (!solid.faces[face].active)
            continue;
        const auto found = std::find_if(grazed.begin(), grazed.end(), [face](const auto& item) { return item.first == face; });
        intersect_face(result, solid, face, frame, met, found == grazed.end() ? nullptr : &found->second);
    }
    std::sort(result.hits.begin(), result.hits.end(), [](const RayHit& a, const RayHit& b) {
        return a.distance < b.distance;
    });
    return result;
}

State ray_state(const RayResult& result, double tolerance) {
    for (const RayHit& hit : result.hits) {
        if (hit.distance_enclosure[1] <= tolerance)
            return State::Boundary;
        if (hit.distance_enclosure[0] <= tolerance)
            return State::Unknown;
    }
    if (!result.complete)
        return State::Unknown;
    int winding = 0;
    double previous = -std::numeric_limits<double>::infinity();
    for (const RayHit& hit : result.hits) {
        const double separated = std::nextafter(previous + tolerance, std::numeric_limits<double>::infinity());
        if (hit.trim != State::Inside || hit.crossing == 0 || hit.distance_enclosure[0] <= separated)
            return State::Unknown;
        previous = hit.distance_enclosure[1];
        winding += hit.crossing;
    }
    return winding == 0 ? State::Outside : State::Inside;
}

bool near_edge(const BRep& brep, int face, const Point& query, double tolerance) {
    for (const BRepRef& wire : brep.m_faces[face].wires)
        for (const BRepRef& use : brep.wire_edges(wire)) {
            const BRepEdge& edge = brep.m_edges[use.index];
            if (brep.m_vertices[edge.start_vertex].point.distance(query) <= tolerance)
                return true;
            if (edge.degenerated)
                continue;
            const NurbsCurve& curve = brep.m_curves_3d[edge.curve_3d_index];
            const double parameter = Closest::curve_point(curve, query).first;
            if (curve.point_at(parameter).distance(query) <= tolerance)
                return true;
        }
    return false;
}

State boundary_state(const Solid& solid, const Point& query, double tolerance, std::vector<std::pair<int, Vector>>& grazed) {
    const BRep& brep = solid.brep;
    const xyz q = query;
    for (int face = 0; face < brep.face_count(); ++face) {
        const int support = brep.m_faces[face].surface_index;
        const std::array<xyz, 2>& box = solid.boxes[support];
        if (!solid.faces[face].active || q[0] < box[0][0] - tolerance || q[0] > box[1][0] + tolerance ||
            q[1] < box[0][1] - tolerance || q[1] > box[1][1] + tolerance ||
            q[2] < box[0][2] - tolerance || q[2] > box[1][2] + tolerance)
            continue;
        const NurbsSurface& surface = brep.m_surfaces[support];
        const std::tuple<double, double, double> nearest = Closest::surface_point(surface, query);
        const double u = std::get<0>(nearest);
        const double v = std::get<1>(nearest);
        if (surface.point(u, v).distance(q) > tolerance)
            continue;
        std::array<xyz, 6> values;
        surface.jet(u, v, 1, values);
        const double speed = std::max(values[1].length(), values[2].length());
        if (!(speed > 0.0))
            return near_edge(brep, face, query, tolerance) ? State::Boundary : State::Unknown;
        const State trim = face_uv(solid, face, {u, v, 0.0}, tolerance / speed);
        if (trim == State::Inside || near_edge(brep, face, query, tolerance))
            return State::Boundary;
        if (trim == State::Unknown)
            return State::Unknown;
        Vector normal(values[1].cross(values[2]));
        if (normal.normalize_self())
            grazed.push_back({face, normal});
    }
    return State::Outside;
}

}

Solid::Solid(const BRep& model, bool boundaries) : brep(model), solid(model.is_solid()), valid(solid || model.is_valid()) {
    if (!valid)
        return;
    faces.resize(brep.face_count());
    // The orientation BRep::face_orientation finds: the first use in a solid's shell, else the first in any shell.
    std::vector<bool> oriented(faces.size(), false);
    for (const BRepSolid& body : brep.m_solids)
        for (const BRepRef& shell : body.shells)
            for (const BRepRef& face : brep.m_shells[shell.index].faces) {
                const BRepOrientation orientation = brep_compose(shell.orientation, face.orientation);
                if (orientation != BRepOrientation::Internal && orientation != BRepOrientation::External)
                    faces[face.index].active = true;
                if (!oriented[face.index])
                    faces[face.index].orientation = orientation;
                oriented[face.index] = true;
            }
    for (const BRepShell& shell : brep.m_shells)
        for (const BRepRef& face : shell.faces) {
            if (!oriented[face.index])
                faces[face.index].orientation = face.orientation;
            oriented[face.index] = true;
        }
    boxes.resize(brep.m_surfaces.size());
    for (size_t k = 0; k < boxes.size(); ++k) {
        const NurbsSurface& surface = brep.m_surfaces[k];
        xyz lo = surface.get_cv(0, 0), hi = lo;
        for (int i = 0; i < surface.cv_count(0); ++i)
            for (int j = 0; j < surface.cv_count(1); ++j) {
                const xyz p = surface.get_cv(i, j);
                for (int d = 0; d < 3; ++d) {
                    lo[d] = std::min(lo[d], p[d]);
                    hi[d] = std::max(hi[d], p[d]);
                }
            }
        boxes[k] = {lo, hi};
    }
    for (int i = 0; i < brep.face_count(); ++i) {
        Face& face = faces[i];
        face.active = face.active || brep.m_solids.empty();
        if (boundaries)
            lay_out(brep, i, face);
    }
}

double clear_ray_height(const std::vector<const NurbsCurve*>& loop, double height, double tolerance) {
    std::vector<std::array<double, 2>> ends;
    for (const NurbsCurve* curve : loop)
        ends.push_back(end_heights(*curve, false));
    return clear_height(ends, height, tolerance);
}

State face_uv(const BRep& brep, int face, const Point& uv, double tolerance) {
    check_tolerance(tolerance);
    if (face < 0 || face >= brep.face_count() || !finite(uv) || !brep.is_valid())
        throw std::invalid_argument("BRep classification requires a valid face and finite UV point");
    const auto boundary = brep_face_boundary::extract(brep, face);
    if (!boundary.complete)
        return State::Unknown;
    return face_state(boundary, nullptr, uv, tolerance);
}

State face_uv(const brep_face_boundary::Result& boundary, const Point& uv, double tolerance) {
    return face_state(boundary, nullptr, uv, tolerance);
}

State face_uv(const Solid& solid, int face, const Point& uv, double tolerance) {
    check_tolerance(tolerance);
    if (face < 0 || face >= static_cast<int>(solid.faces.size()) || !finite(uv))
        throw std::invalid_argument("BRep classification requires a valid face and finite UV point");
    Face found;
    if (!solid.faces[face].boundary.owner)
        lay_out(solid.brep, face, found);
    const Face& prepared = solid.faces[face].boundary.owner ? solid.faces[face] : found;
    if (!prepared.boundary.complete)
        return State::Unknown;
    return face_state(prepared.boundary, &prepared.ends, uv, tolerance);
}

RayResult ray(const BRep& brep, const Point& origin, const Vector& direction, double tolerance) {
    check_tolerance(tolerance);
    return ray(Solid(brep, false), origin, direction, tolerance);
}

RayResult ray(const Solid& solid, const Point& origin, const Vector& direction, double tolerance) {
    check_tolerance(tolerance);
    if (!finite(origin) || !std::isfinite(direction[0]) || !std::isfinite(direction[1]) ||
        !std::isfinite(direction[2]) || std::max({std::abs(direction[0]), std::abs(direction[1]),
            std::abs(direction[2])}) == 0.0 || !solid.valid)
        throw std::invalid_argument("BRep ray requires valid geometry and a finite nonzero direction");
    return cast(solid, {origin, direction, tolerance});
}

State point(const BRep& brep, const Point& query, double tolerance) {
    check_tolerance(tolerance);
    return point(Solid(brep, false), query, tolerance);
}

State point(const Solid& solid, const Point& query, double tolerance) {
    check_tolerance(tolerance);
    if (!finite(query))
        throw std::invalid_argument("BRep classification requires a finite point");
    if (solid.brep.m_faces.empty() && solid.brep.m_solids.empty())
        return State::Outside;
    if (!solid.solid)
        throw std::invalid_argument("BRep solid classification requires closed oriented solids");
    std::vector<std::pair<int, Vector>> grazed;
    const State boundary = boundary_state(solid, query, tolerance, grazed);
    if (boundary != State::Outside)
        return boundary;
    const std::array<Vector, 7> directions = {
        Vector(1.0, 0.371, 0.529), Vector(-0.217, 1.0, 0.413), Vector(0.319, -0.271, 1.0),
        Vector(-1.0, 0.723, -0.219), Vector(0.613, -1.0, -0.417), Vector(-0.531, -0.337, -1.0), Vector(1.0, 1.0, 1.0)
    };
    State previous = State::Unknown;
    for (const Vector& direction : directions) {
        const State current = ray_state(cast(solid, {query, direction, tolerance}, grazed), tolerance);
        if (current == State::Boundary)
            return current;
        if (current == State::Unknown)
            continue;
        if (previous != State::Unknown)
            return current == previous ? current : State::Unknown;
        previous = current;
    }
    return State::Unknown;
}

}
