#include "brep_classify.h"
#include "brep_face_boundary.h"
#include "nurbs_interval.h"
#include "brep_curve_use.h"
#include "closest.h"
#include "surface_ray.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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

bool finite(const Point& p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

double segment_distance(const Point& p, const Point& a, const Point& b) {
    const Vector delta = b - a;
    const double squared = delta.dot(delta);
    const double t = squared > 0.0 ? std::clamp((p - a).dot(delta) / squared, 0.0, 1.0) : 0.0;
    return p.distance(a + delta * t);
}

int crossing(const Point& a, const Point& b, const Point& query) {
    if ((a[1] > query[1]) == (b[1] > query[1]))
        return 0;
    const double t = (query[1] - a[1]) / (b[1] - a[1]);
    if (a[0] + t * (b[0] - a[0]) <= query[0])
        return 0;
    return b[1] > a[1] ? 1 : -1;
}

State curve_winding(const NurbsCurve& curve, const Point& query, double height, double tolerance, int& winding) {
    const Point ray(query[0], height, 0.0);
    std::vector<CurvePart> work = {{curve, 0}};
    for (size_t iteration = 0; !work.empty() && iteration < 100000; ++iteration) {
        CurvePart part = std::move(work.back());
        work.pop_back();
        const Point a = part.curve.point_at_start();
        const Point b = part.curve.point_at_end();
        Point lo = a;
        Point hi = a;
        double error = 0.0;
        for (int i = 0; i < part.curve.cv_count(); ++i) {
            const Point p = part.curve.get_cv(i);
            if (!finite(p) || !std::isfinite(part.curve.weight(i)) || !(part.curve.weight(i) > 0.0))
                return State::Unknown;
            for (int d = 0; d < 2; ++d) {
                lo[d] = std::min(lo[d], p[d]);
                hi[d] = std::max(hi[d], p[d]);
            }
            error = std::max(error, segment_distance(p, a, b));
        }
        if (hi[0] < query[0] - tolerance || hi[1] < query[1] - tolerance || lo[1] > query[1] + tolerance)
            continue;
        if (lo[0] > query[0] + tolerance) {
            winding += crossing(a, b, ray);
            continue;
        }
        if (error <= tolerance * 0.125) {
            if (segment_distance(query, a, b) <= tolerance + error)
                return State::Boundary;
            winding += crossing(a, b, ray);
            continue;
        }
        const double middle = std::midpoint(part.curve.domain_start(), part.curve.domain_end());
        NurbsCurve left;
        NurbsCurve right;
        if (part.depth >= 64 || !part.curve.split(middle, left, right))
            return State::Unknown;
        work.push_back({std::move(right), part.depth + 1});
        work.push_back({std::move(left), part.depth + 1});
    }
    return work.empty() ? State::Outside : State::Unknown;
}

State ranged_winding(const NurbsCurve& curve, std::pair<double,double> range,
    const Point& query, double height, double tolerance, int& winding) {
    struct Part { double first, last; int depth; };
    std::vector<Part> work{{range.first,range.second,0}};
    for (size_t iteration=0; !work.empty() && iteration<100000; ++iteration) {
        const auto part=work.back();
        work.pop_back();
        nurbs_interval::CurveBounds bounds;
        if (!nurbs_interval::curve_bounds(curve,{part.first,part.last},{0,0,0},bounds))
            return State::Unknown;
        const auto x=bounds.point[0], y=bounds.point[1];
        const Point a=curve.point_at(part.first), b=curve.point_at(part.last);
        if (!finite(a) || !finite(b))
            return State::Unknown;
        if (x.hi<query[0]-tolerance || y.hi<query[1]-tolerance || y.lo>query[1]+tolerance)
            continue;
        if (x.lo>query[0]+tolerance) {
            winding+=crossing(a,b,{query[0],height,0});
            continue;
        }
        double error=0;
        for (double u : {x.lo,x.hi})
            for (double v : {y.lo,y.hi})
                error=std::max(error,segment_distance({u,v,0},a,b));
        if (error<=tolerance*.125) {
            if (segment_distance(query,a,b)<=tolerance+error)
                return State::Boundary;
            winding+=crossing(a,b,{query[0],height,0});
            continue;
        }
        const double middle=std::midpoint(part.first,part.last);
        if (part.depth>=64 || !(middle>part.first && middle<part.last))
            return State::Unknown;
        work.push_back({middle,part.last,part.depth+1});
        work.push_back({part.first,middle,part.depth+1});
    }
    return work.empty()?State::Outside:State::Unknown;
}

double use_ray_height(const BRep& brep, const brep_face_boundary::Wire& wire,
    double height, double tolerance) {
    for (double share : {0.0, .25, -.25, .125, -.125, .1875, -.1875, .0625, -.0625}) {
        const double candidate=height+share*tolerance;
        const double margin=64*std::numeric_limits<double>::epsilon()*std::max(1.,std::abs(candidate));
        bool clear=true;
        for (const auto& use : wire.uses) {
            if (use.pcurve<0 || use.pcurve>=static_cast<int>(brep.m_curves_2d.size()))
                continue;
            const auto& curve=brep.m_curves_2d[use.pcurve];
            for (double t : {use.pcurve_domain.first,use.pcurve_domain.second})
                clear=clear && std::abs(curve.point_at(t)[1]-candidate)>margin;
        }
        if (clear)
            return candidate;
    }
    return height;
}

State wire_state(const BRep& brep, const brep_face_boundary::Wire& wire, const Point& uv, double tolerance) {
    const double height = use_ray_height(brep, wire, uv[1], tolerance);
    int winding = 0;
    for (const auto& use : wire.uses) {
        const auto& edge = use.edge;
        if (edge.orientation == BRepOrientation::Internal || edge.orientation == BRepOrientation::External)
            continue;
        const int ci = use.pcurve;
        if (ci < 0 || ci >= static_cast<int>(brep.m_curves_2d.size()))
            return State::Unknown;
        int contribution = 0;
        const auto& curve=brep.m_curves_2d[ci];
        const State state = use.pcurve_domain==curve.domain()
            ? curve_winding(curve,uv,height,tolerance,contribution)
            : ranged_winding(curve,use.pcurve_domain,uv,height,tolerance,contribution);
        if (state == State::Boundary || state == State::Unknown)
            return state;
        winding += edge.orientation == BRepOrientation::Reversed ? -contribution : contribution;
    }
    return winding == 0 ? State::Outside : State::Inside;
}

bool append_hit(RayResult& result, const BRep& brep, int face,
    const surface_ray::Root& root, const RayFrame& frame) {
    const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
    const std::vector<Vector> values = surface.evaluate(root.u, root.v, 1);
    if (values.size() != 3)
        return false;
    const double speed = std::max(values[1].magnitude(), values[2].magnitude());
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
    const State trim = face_uv(brep, face, {root.u, root.v, 0.0}, tolerance);
    if (trim == State::Unknown)
        return false;
    if (trim == State::Outside)
        return true;
    const int crossing = brep.face_orientation(face) == BRepOrientation::Reversed ? -root.crossing : root.crossing;
    result.hits.push_back({root.point, root.distance, root.u, root.v, face, crossing, trim, root.distance_enclosure});
    return true;
}

void intersect_face(RayResult& result, const BRep& brep, int face, const RayFrame& frame) {
    const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
    surface_ray::Options options;
    options.include_origin = true;
    const auto intersections = surface_ray::intersect(surface, frame.origin, frame.direction, frame.tolerance, options);
    if (!intersections.complete) {
        result.complete = false;
        result.failure = "Unresolved ray on face " + std::to_string(face) + ": " + intersections.failure;
        result.unresolved_face = face;
        result.unresolved_domain = {surface.domain(0).first, surface.domain(0).second,
            surface.domain(1).first, surface.domain(1).second};
    }
    for (const auto& root : intersections.roots)
        if (!append_hit(result, brep, face, root, frame)) {
            result.complete = false;
            result.failure = "Unresolved ray trim on face " + std::to_string(face);
            result.unresolved_face = face;
            result.unresolved_domain = root.enclosure;
        }
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

std::vector<bool> active_faces(const BRep& brep) {
    std::vector<bool> active(brep.face_count(), brep.m_solids.empty());
    for (const BRepSolid& solid : brep.m_solids)
        for (const BRepRef& shell : solid.shells)
            for (const BRepRef& face : brep.m_shells[shell.index].faces) {
                const BRepOrientation orientation = brep_compose(shell.orientation, face.orientation);
                if (orientation != BRepOrientation::Internal && orientation != BRepOrientation::External)
                    active[face.index] = true;
            }
    return active;
}

bool near_surface_bounds(const NurbsSurface& surface, const Point& query, double tolerance) {
    Point lo = surface.get_cv(0, 0);
    Point hi = lo;
    for (int i = 0; i < surface.cv_count(0); ++i)
        for (int j = 0; j < surface.cv_count(1); ++j) {
            const Point p = surface.get_cv(i, j);
            for (int d = 0; d < 3; ++d) {
                lo[d] = std::min(lo[d], p[d]);
                hi[d] = std::max(hi[d], p[d]);
            }
        }
    for (int d = 0; d < 3; ++d)
        if (query[d] < lo[d] - tolerance || query[d] > hi[d] + tolerance)
            return false;
    return true;
}

State ranged_near_edge(const NurbsCurve& curve, std::pair<double,double> range,
    const Point& query, double tolerance) {
    struct Part { double first, last; int depth; };
    std::vector<Part> work{{range.first,range.second,0}};
    const auto limit=nurbs_interval::Interval(tolerance)*nurbs_interval::Interval(tolerance);
    for (size_t iteration=0; !work.empty() && iteration<100000; ++iteration) {
        const auto part=work.back();
        work.pop_back();
        nurbs_interval::CurveBounds bounds;
        if (!nurbs_interval::curve_bounds(curve,{part.first,part.last},
                {query[0],query[1],query[2]},bounds))
            return State::Unknown;
        nurbs_interval::Interval squared;
        for (const auto& coordinate : bounds.point) {
            const double distance=coordinate.lo>0?coordinate.lo:coordinate.hi<0?-coordinate.hi:0;
            squared=squared+nurbs_interval::Interval(distance)*nurbs_interval::Interval(distance);
        }
        if (squared.lo>limit.hi)
            continue;
        for (double t : {part.first,part.last})
            if (curve.point_at(t).distance(query)<=tolerance)
                return State::Boundary;
        const double middle=std::midpoint(part.first,part.last);
        if (part.depth>=64 || !(middle>part.first && middle<part.last))
            return State::Unknown;
        work.push_back({middle,part.last,part.depth+1});
        work.push_back({part.first,middle,part.depth+1});
    }
    return work.empty()?State::Outside:State::Unknown;
}

bool near_edge(const BRep& brep, int face, const Point& query, double tolerance, bool& unresolved) {
    for (const BRepRef& wire : brep.m_faces[face].wires)
        for (const BRepRef& use : brep.wire_edges(wire)) {
            const BRepEdge& edge = brep.m_edges[use.index];
            if (brep.m_vertices[edge.start_vertex].point.distance(query) <= tolerance)
                return true;
            if (edge.degenerated)
                continue;
            const NurbsCurve& curve = brep.m_curves_3d[edge.curve_3d_index];
            const auto range=brep.edge_domain(use.index);
            if (range!=curve.domain()) {
                const State state=ranged_near_edge(curve,range,query,tolerance);
                if (state==State::Boundary)
                    return true;
                unresolved=unresolved || state==State::Unknown;
                continue;
            }
            const double parameter = Closest::curve_point(curve, query).first;
            if (curve.point_at(parameter).distance(query) <= tolerance)
                return true;
        }
    return false;
}

State boundary_state(const BRep& brep, const Point& query, double tolerance) {
    const std::vector<bool> active = active_faces(brep);
    for (int face = 0; face < brep.face_count(); ++face) {
        const NurbsSurface& surface = brep.m_surfaces[brep.m_faces[face].surface_index];
        if (!active[face] || !near_surface_bounds(surface, query, tolerance))
            continue;
        const std::tuple<double, double, double> nearest = Closest::surface_point(surface, query);
        const double u = std::get<0>(nearest);
        const double v = std::get<1>(nearest);
        if (surface.point_at(u, v).distance(query) > tolerance)
            continue;
        const std::vector<Vector> values = surface.evaluate(u, v, 1);
        const double speed = std::max(values[1].magnitude(), values[2].magnitude());
        bool edge_unresolved=false;
        if (!(speed > 0.0))
            return near_edge(brep, face, query, tolerance,edge_unresolved) ? State::Boundary : State::Unknown;
        const State trim = face_uv(brep, face, {u, v, 0.0}, tolerance / speed);
        if (trim == State::Inside || near_edge(brep, face, query, tolerance,edge_unresolved))
            return State::Boundary;
        if (trim == State::Unknown || edge_unresolved)
            return State::Unknown;
    }
    return State::Outside;
}

}

double clear_ray_height(const std::vector<const NurbsCurve*>& loop, double height, double tolerance) {
    const auto clear = [&loop](double candidate) {
        const double margin = 64.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(candidate));
        for (const NurbsCurve* curve : loop)
            if (std::abs(curve->point_at_start()[1] - candidate) <= margin ||
                std::abs(curve->point_at_end()[1] - candidate) <= margin)
                return false;
        return true;
    };
    for (double share : {0.0, 0.25, -0.25, 0.125, -0.125, 0.1875, -0.1875, 0.0625, -0.0625})
        if (clear(height + share * tolerance))
            return height + share * tolerance;
    return height;
}

State face_uv(const BRep& brep, int face, const Point& uv, double tolerance) {
    check_tolerance(tolerance);
    if (face < 0 || face >= brep.face_count() || !finite(uv) || !brep.is_valid())
        throw std::invalid_argument("BRep classification requires a valid face and finite UV point");
    const auto boundary = brep_face_boundary::extract(brep, face);
    if (!boundary.complete)
        return State::Unknown;
    return face_uv(boundary, uv, tolerance);
}

State face_uv(const brep_face_boundary::Result& boundary, const Point& uv, double tolerance) {
    check_tolerance(tolerance);
    if (!finite(uv))
        throw std::invalid_argument("BRep classification requires a finite UV point");
    if (!boundary.complete || !boundary.owner || boundary.wires.empty() ||
        boundary.face < 0 || boundary.face >= boundary.owner->face_count())
        return State::Unknown;
    const BRep& brep = *boundary.owner;
    const State outer = wire_state(brep, boundary.wires[0], uv, tolerance);
    if (outer != State::Inside)
        return outer;
    for (size_t i = 1; i < boundary.wires.size(); ++i) {
        const State hole = wire_state(brep, boundary.wires[i], uv, tolerance);
        if (hole == State::Unknown || hole == State::Boundary)
            return hole;
        if (hole == State::Inside)
            return State::Outside;
    }
    return State::Inside;
}

RayResult ray(const BRep& brep, const Point& origin, const Vector& direction, double tolerance) {
    check_tolerance(tolerance);
    if (!finite(origin) || !std::isfinite(direction[0]) || !std::isfinite(direction[1]) ||
        !std::isfinite(direction[2]) || std::max({std::abs(direction[0]), std::abs(direction[1]),
            std::abs(direction[2])}) == 0.0 || !brep.is_valid())
        throw std::invalid_argument("BRep ray requires valid geometry and a finite nonzero direction");
    const RayFrame frame = {origin, direction, tolerance};
    RayResult result;
    const std::vector<bool> active = active_faces(brep);
    for (int face = 0; face < brep.face_count(); ++face)
        if (active[face])
            intersect_face(result, brep, face, frame);
    std::sort(result.hits.begin(), result.hits.end(), [](const RayHit& a, const RayHit& b) {
        return a.distance < b.distance;
    });
    return result;
}

State point(const BRep& brep, const Point& query, double tolerance) {
    check_tolerance(tolerance);
    if (!finite(query))
        throw std::invalid_argument("BRep classification requires a finite point");
    if (brep.m_faces.empty() && brep.m_solids.empty())
        return State::Outside;
    if (!brep.is_solid())
        throw std::invalid_argument("BRep solid classification requires closed oriented solids");
    const State boundary = boundary_state(brep, query, tolerance);
    if (boundary != State::Outside)
        return boundary;
    const std::array<Vector, 7> directions = {
        Vector(1.0, 0.371, 0.529), Vector(-0.217, 1.0, 0.413), Vector(0.319, -0.271, 1.0),
        Vector(-1.0, 0.723, -0.219), Vector(0.613, -1.0, -0.417), Vector(-0.531, -0.337, -1.0), Vector(1.0, 1.0, 1.0)
    };
    State previous = State::Unknown;
    for (const Vector& direction : directions) {
        const State current = ray_state(ray(brep, query, direction, tolerance), tolerance);
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
