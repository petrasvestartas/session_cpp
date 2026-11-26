#include "bop_ds.h"
#include "bop_ff.h"
#include "curve_intersection.h"
#include "curve_surface_intersection.h"
#include "intersection_analytic.h"
#include "nurbs_implicit.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <numbers>
#include <numeric>
#include <tuple>

namespace session_cpp::bop {
namespace {

constexpr double confusion = 1e-7;

Box curve_box(const NurbsCurve& c, double grow) {
    Box box;
    for (int i = 0; i < c.cv_count(); ++i)
        box.add(c.get_cv(i), grow);
    return box;
}

Box surface_box(const NurbsSurface& s, double grow) {
    Box box;
    for (int i = 0; i < s.cv_count(0); ++i)
        for (int j = 0; j < s.cv_count(1); ++j)
            box.add(s.get_cv(i, j), grow);
    return box;
}

/// The surface cut down to the uv box of a face, a little wider; whole where the face wraps a period
NurbsSurface patch(const Surface& form, const Region& region) {
    if (form.kind() != Surface::Kind::Nurbs) {
        // An analytic surface has no bounds of its own: over the box of the face, an angle over no more than its turn.
        Surface cut = form;
        for (int d = 0; d < 2 && !region.loops.empty(); ++d) {
            const double pad = 0.02 * (region.hi[d] - region.lo[d]), period = form.period(d);
            double a = region.lo[d] - pad, b = region.hi[d] + pad;
            if (period > 0.0 && b - a >= period)
                a = region.lo[d], b = a + period;
            if (form.kind() == Surface::Kind::Sphere && d == 1)
                a = std::max(a, -0.5 * std::numbers::pi), b = std::min(b, 0.5 * std::numbers::pi);
            cut.set_domain(d, a, b);
        }
        return cut.nurbs();
    }
    const NurbsSurface& s = form.nurbs();
    NurbsSurface result = s;
    for (int d = 0; d < 2; ++d) {
        const auto [lo, hi] = s.domain(d);
        const double pad = 0.02 * (region.hi[d] - region.lo[d]) + 1e-9 * (hi - lo);
        const double a = std::max(lo, region.lo[d] - pad), b = std::min(hi, region.hi[d] + pad);
        if (region.loops.empty() || region.lo[d] < lo - pad || region.hi[d] > hi + pad || b - a >= (1 - 1e-9) * (hi - lo) || b <= a)
            continue;
        NurbsSurface cut = result;
        if (cut.trim(d, {a, b}))
            result = cut;
    }
    return result;
}

std::vector<int> active_faces(const BRep& brep) {
    std::vector<int> result;
    for (const BRepSolid& solid : brep.m_solids)
        for (const BRepRef& shell : solid.shells)
            for (const BRepRef& face : brep.m_shells[shell.index].faces)
                result.push_back(face.index);
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

struct Filler {
    DS& ds;
    std::array<std::vector<int>, 2> act_v, act_e, act_f;
    std::map<int, Curve> edge_forms;
    std::map<int, Surface> face_forms;

    /// The line, circle or ellipse an edge is to rounding, recognized while the pools hold freeform curves
    const Curve& form(int e) {
        const Curve& c = ds.curve_of(ds.edges[e]);
        if (c.kind() != Curve::Kind::Nurbs)
            return c;
        const auto found = edge_forms.find(e);
        return found != edge_forms.end() ? found->second : edge_forms[e] = Curve::recognized(c.nurbs());
    }

    /// The plane, cylinder, cone, sphere or torus a face's surface is to rounding
    const Surface& form(const Face& f, int id) {
        if (ds.surface_of(f).kind() != Surface::Kind::Nurbs)
            return ds.surface_of(f);
        const auto found = face_forms.find(id);
        if (found != face_forms.end())
            return found->second;
        const NurbsSurface& s = ds.surface_of(f).nurbs();
        const Box box = surface_box(s, 0.0);
        return face_forms[id] = Surface::recognized(s, 1e-10 * Point(box.lo[0], box.lo[1], box.lo[2]).distance(Point(box.hi[0], box.hi[1], box.hi[2])));
    }

    /// IntAna_IntConicQuad for an edge and a face that have forms: the clean crossings of the edge's range with the whole surface. False where there is no closed form, where the edge touches the surface or meets it at a shallow angle, or comes within the tolerance of it without crossing: those are the tolerant intersection's.
    bool closed_form_ef(int e, int fid, double tolerance, curve_surface_intersection::Result& r) {
        const Edge& edge = ds.edges[e];
        const Curve& curve = form(e);
        const Surface& surface = form(ds.faces[fid], fid);
        if (curve.kind() == Curve::Kind::Nurbs || surface.kind() == Surface::Kind::Nurbs)
            return false;
        const intersection_analytic::Result exact = intersection_analytic::curve_surface(curve, surface, tolerance);
        if (!exact.done || exact.coincident)
            return false;
        const Curve& c = ds.curve_of(edge);
        const Surface& s = ds.surface_of(ds.faces[fid]);
        const auto [d0, d1] = curve.domain();
        std::vector<curve_surface_intersection::Hit> hits;
        for (const intersection_analytic::Hit& hit : exact.points) {
            if (hit.t < d0 - 1e-9 * (d1 - d0) || hit.t > d1 + 1e-9 * (d1 - d0))
                continue;
            const double t = parameter(c, hit.point, edge.t0, edge.t1);
            const auto [u, v] = s.closest_parameters(hit.point);
            Vector along = c.tangent_at(t), normal = s.normal_at(u, v);
            along.normalize_self();
            normal.normalize_self();
            if (hit.tangent || c.point_at(t).distance(hit.point) > tolerance || s.point_at(u, v).distance(hit.point) > tolerance || std::abs(along.dot(normal)) < 0.02)
                return false;
            hits.push_back({hit.point, t, u, v, false});
        }
        // Where the edge comes nearest to the surface without a crossing there, it must stay clear of the tolerance.
        const int n = 32;
        const auto gap = [&](double t) { return surface.distance(c.point_at(t)); };
        std::vector<double> d(n + 1);
        for (int i = 0; i <= n; ++i)
            d[i] = gap(edge.t0 + (edge.t1 - edge.t0) * i / n);
        for (int i = 0; i <= n; ++i) {
            if ((i > 0 && d[i] > d[i - 1]) || (i < n && d[i] > d[i + 1]))
                continue;
            double lo = edge.t0 + (edge.t1 - edge.t0) * std::max(i - 1, 0) / n, hi = edge.t0 + (edge.t1 - edge.t0) * std::min(i + 1, n) / n;
            for (int k = 0; k < 50; ++k) {
                const double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
                (gap(m1) < gap(m2) ? hi : lo) = gap(m1) < gap(m2) ? m2 : m1;
            }
            const Point p = c.point_at(0.5 * (lo + hi));
            bool crossing = surface.distance(p) > tolerance;
            for (const curve_surface_intersection::Hit& hit : hits)
                crossing = crossing || hit.point.distance(p) <= 100.0 * tolerance;
            if (!crossing)
                return false;
        }
        r.points = hits;
        return true;
    }

    /// The point of an edge nearest to p
    Point nearest(const Edge& e, const Point& p) const { return ds.curve_of(e).point_at(parameter(ds.curve_of(e), p, e.t0, e.t1)); }

    double tol(int v) const { return ds.vertices[ds.real(v)].tolerance; }
    const Point& at(int v) const { return ds.vertices[ds.real(v)].point; }

    /// How far an edge's curve runs from a face it bounds: the edge tolerance of BRepLib::SameParameter, which loose files do not carry
    static double deviation(const BRep& brep, const BRepRef& use, int face) {
        const Span p = pcurve_span(brep, use.index, face, use.orientation);
        if (brep.m_edges[use.index].degenerated || !p.curve)
            return 0.0;
        const Span c = edge_span(brep, use.index);
        const Surface& s = brep.m_surfaces[brep.m_faces[face].surface_index];
        double worst = 0.0;
        for (int i = 1; i < 8; ++i) {
            const Point q = c.at(i / 8.0);
            const Point uv = p.at(i / 8.0);
            double d = s.point_at(uv[0], uv[1]).distance(q);
            if (d > std::max(worst, confusion))
                d = std::min(d, s.closest_point(q).distance(q));
            worst = std::max(worst, d);
        }
        return worst;
    }

    void init(const BRep& a, const BRep& b) {
        ds.operands = {&a, &b};
        for (int o = 0; o < 2; ++o) {
            const BRep& brep = ds.brep(o);
            ds.vertex_offset[o] = static_cast<int>(ds.vertices.size());
            ds.edge_offset[o] = static_cast<int>(ds.edges.size());
            ds.face_offset[o] = static_cast<int>(ds.faces.size());
            for (int i = 0; i < brep.vertex_count(); ++i)
                ds.vertices.push_back({brep.m_vertices[i].point, std::max(brep.m_vertices[i].tolerance, confusion), o, i, -1, 0.0, false});
            for (int i = 0; i < brep.edge_count(); ++i) {
                const BRepEdge& e = brep.m_edges[i];
                Edge edge;
                edge.operand = o;
                edge.local = i;
                edge.tolerance = std::max(e.tolerance, confusion);
                edge.degenerated = e.degenerated;
                if (e.degenerated)
                    std::tie(edge.t0, edge.t1) = brep.pcurve_range(e.pcurves.at(0));
                else {
                    std::tie(edge.t0, edge.t1) = brep.edge_range(i);
                    if (e.range[0] < e.range[1])
                        edge.piece = std::make_shared<const Curve>(brep.m_curves_3d[e.curve_3d_index].kind() == Curve::Kind::Nurbs ? Curve(brep.edge_nurbs(i)) : brep.edge_curve(i));
                }
                edge.paves = {{ds.vertex_offset[o] + e.start_vertex, edge.t0}, {ds.vertex_offset[o] + e.end_vertex, edge.t1}};
                ds.edges.push_back(std::move(edge));
            }
            for (int i = 0; i < brep.face_count(); ++i) {
                ds.faces.emplace_back();
                ds.faces.back().operand = o;
                ds.faces.back().local = i;
                ds.faces.back().tolerance = std::max(brep.m_faces[i].tolerance, confusion);
            }
            std::vector<bool> used_e(brep.edge_count(), false), used_v(brep.vertex_count(), false);
            act_f[o] = active_faces(brep);
            for (int f : act_f[o]) {
                Face& face = ds.faces[ds.face_offset[o] + f];
                for (const BRepRef& wire : brep.m_faces[f].wires)
                    for (const BRepRef& use : brep.wire_edges(wire)) {
                        const int e = ds.edge_offset[o] + use.index;
                        if (std::find(face.edges.begin(), face.edges.end(), e) == face.edges.end())
                            face.edges.push_back(e);
                        else if (std::find(face.seams.begin(), face.seams.end(), e) == face.seams.end())
                            face.seams.push_back(e);
                        ds.edges[e].tolerance = std::max({ds.edges[e].tolerance, face.tolerance, deviation(brep, use, f)});
                        used_e[use.index] = true;
                    }
                face.region = face_region(brep, f, face.tolerance);
                double grow = face.tolerance;
                for (int e : face.edges)
                    grow = std::max(grow, ds.edges[e].tolerance);
                face.patch = patch(ds.surface_of(face), face.region);
                face.box = surface_box(face.patch, grow);
            }
            for (int i = 0; i < brep.edge_count(); ++i) {
                if (!used_e[i])
                    continue;
                act_e[o].push_back(ds.edge_offset[o] + i);
                Edge& edge = ds.edges[ds.edge_offset[o] + i];
                for (const Pave& p : edge.paves) {
                    used_v[ds.vertices[p.vertex].local] = true;
                    ds.vertices[p.vertex].tolerance = std::max(ds.vertices[p.vertex].tolerance, edge.tolerance);
                }
                if (!edge.degenerated)
                    edge.box = curve_box(ds.curve_of(edge).nurbs(), edge.tolerance);
            }
            for (int i = 0; i < brep.vertex_count(); ++i)
                if (used_v[i])
                    act_v[o].push_back(ds.vertex_offset[o] + i);
        }
    }

    /// The real vertices of an operand, each once
    std::vector<int> real_vertices(int o) const {
        std::vector<int> result;
        for (int v : act_v[o])
            result.push_back(ds.real(v));
        std::sort(result.begin(), result.end());
        result.erase(std::unique(result.begin(), result.end()), result.end());
        return result;
    }

    bool has_pave(int e, int v) const {
        for (const Pave& p : ds.edges[e].paves)
            if (ds.real(p.vertex) == ds.real(v))
                return true;
        return false;
    }

    void add_pave(int e, int v, double t) {
        if (!has_pave(e, v))
            ds.edges[e].paves.push_back({v, std::clamp(t, ds.edges[e].t0, ds.edges[e].t1)});
    }

    void grow(int v, double tolerance) {
        Vertex& vertex = ds.vertices[ds.real(v)];
        vertex.tolerance = std::max(vertex.tolerance, tolerance);
    }

    /// The vertex among the candidates whose ball, grown by the tolerance, holds the point
    int find_vertex(const std::vector<int>& candidates, const Point& p, double tolerance) const {
        int best = -1;
        double nearest = 1e300;
        for (int v : candidates) {
            const double d = at(v).distance(p);
            if (d <= tol(v) + tolerance + ds.fuzzy && d < nearest) {
                nearest = d;
                best = ds.real(v);
            }
        }
        return best;
    }

    std::vector<int> face_vertices(const Face& f) const {
        std::vector<int> result;
        for (int v : f.in_vertices)
            result.push_back(ds.real(v));
        for (int e : f.edges)
            for (const Pave& p : ds.edges[e].paves)
                result.push_back(ds.real(p.vertex));
        return result;
    }

    std::vector<int> edge_vertices(int e) const {
        std::vector<int> result;
        for (const Pave& p : ds.edges[e].paves)
            result.push_back(ds.real(p.vertex));
        return result;
    }

    /// The parameter step over which a crossing leaves the tolerance and a common part does not
    static double probe(const Curve& c, const Edge& edge, double tolerance) {
        const double length = c.point_at(edge.t0).distance(c.point_at(edge.t1)) + c.point_at(edge.t0).distance(c.point_at(0.5 * (edge.t0 + edge.t1)));
        return (edge.t1 - edge.t0) * std::max(1e-2, 4.0 * tolerance / std::max(length, 1e-300));
    }

    /// How far an edge runs, closed or not: its ends by way of its middle
    static double extent(const Curve& c, const Edge& edge) {
        const Point mid = c.point_at(0.5 * (edge.t0 + edge.t1));
        return c.point_at(edge.t0).distance(mid) + mid.distance(c.point_at(edge.t1));
    }

    void perform_vv() {
        std::vector<int> parent(ds.vertices.size());
        std::iota(parent.begin(), parent.end(), 0);
        const auto root = [&](int v) {
            while (parent[v] != v)
                v = parent[v] = parent[parent[v]];
            return v;
        };
        for (int v : act_v[0])
            for (int w : act_v[1])
                if (at(v).distance(at(w)) <= tol(v) + tol(w) + ds.fuzzy)
                    parent[root(v)] = root(w);
        std::map<int, std::vector<int>> groups;
        for (int o = 0; o < 2; ++o)
            for (int v : act_v[o])
                groups[root(v)].push_back(v);
        for (const auto& [unused, group] : groups)
            if (group.size() > 1)
                ds.merge(group);
    }

    void perform_ve() {
        for (int o = 0; o < 2; ++o)
            for (int v : real_vertices(o))
                for (int e : act_e[1 - o]) {
                    Edge& edge = ds.edges[e];
                    Box box;
                    box.add(at(v), tol(v));
                    if (edge.degenerated || !box.overlaps(edge.box) || has_pave(e, v))
                        continue;
                    const Curve& c = ds.curve_of(edge);
                    const double t = parameter(c, at(v), edge.t0, edge.t1);
                    const double d = c.point_at(t).distance(at(v));
                    if (d > tol(v) + edge.tolerance + ds.fuzzy)
                        continue;
                    grow(v, d + edge.tolerance);
                    add_pave(e, v, t);
                }
    }

    void perform_ee() {
        for (int ea : act_e[0])
            for (int eb : act_e[1]) {
                const Edge& a = ds.edges[ea];
                const Edge& b = ds.edges[eb];
                if (a.degenerated || b.degenerated || !a.box.overlaps(b.box))
                    continue;
                const double tolerance = a.tolerance + b.tolerance + ds.fuzzy;
                curve_intersection::Result r;
                try {
                    if (!circles(ea, eb, tolerance, r)) {
                        r = curve_intersection::intersect(ds.curve_of(a).nurbs(), ds.curve_of(b).nurbs(), tolerance);
                        for (curve_intersection::Hit& hit : r.points)
                            hit.a = true_parameter(ds.curve_of(a), hit.a), hit.b = true_parameter(ds.curve_of(b), hit.b);
                    }
                } catch (const std::exception& error) {
                    ds.log.push_back(std::string("EE ") + error.what());
                    continue;
                }
                if (!r.complete)
                    ds.log.push_back("EE incomplete: " + r.failure);
                for (const curve_intersection::Hit& hit : r.points)
                    ee_point(ea, eb, hit.a, hit.b);
                if (!r.complete || tolerance > 1e-6)
                    tolerant_ee(ea, eb, tolerance);
            }
    }

    /// IntAna2d and IntAna_IntConicQuad for two circular edges: where the circles cross cleanly, kept where both arcs hold the point; nothing for two arcs of one circle, which meet in their vertices. False for other curves and where the circles touch or pass within a hundred tolerances without crossing.
    bool circles(int ea, int eb, double tolerance, curve_intersection::Result& r) {
        const Curve &a = form(ea), &b = form(eb);
        if (a.kind() != Curve::Kind::Circle || b.kind() != Curve::Kind::Circle)
            return false;
        const Vector na = a.z(), nb = b.z(), between(b.origin()[0] - a.origin()[0], b.origin()[1] - a.origin()[1], b.origin()[2] - a.origin()[2]);
        const double wide = 100.0 * tolerance, height = between.dot(na);
        std::vector<Point> found;
        if (na.cross(nb).magnitude() > 1e-3) {
            // Where b passes the plane of a: A cos + B sin + C = 0.
            const double A = b.radius() * b.x().dot(na), B = b.radius() * b.y().dot(na), R = std::hypot(A, B);
            if (std::abs(height) > R + wide)
                return true;
            if (std::abs(height) > R - wide)
                return false;
            for (double sign : {-1.0, 1.0}) {
                const Point p = b.point_at(std::atan2(B, A) + sign * std::acos(-height / R));
                const double gap = std::abs(p.distance(a.origin()) - a.radius());
                if (gap > tolerance && gap <= wide)
                    return false;
                if (gap <= tolerance)
                    found.push_back(p);
            }
        } else if (na.cross(nb).magnitude() > 1e-9 || (std::abs(height) > tolerance && std::abs(height) <= wide))
            return false;
        else if (std::abs(height) <= tolerance) {
            Vector along = between - na * height;
            const double d = along.magnitude(), sum = a.radius() + b.radius(), difference = std::abs(a.radius() - b.radius());
            if (d <= tolerance && difference <= tolerance)
                return true;
            if (std::abs(d - sum) <= wide || std::abs(d - difference) <= wide)
                return false;
            if (d < sum && d > difference) {
                along.normalize_self();
                const Vector across = na.cross(along);
                const double k = (a.radius() * a.radius() - b.radius() * b.radius() + d * d) / (2.0 * d), h = std::sqrt(std::max(a.radius() * a.radius() - k * k, 0.0));
                for (double sign : {-1.0, 1.0})
                    found.push_back(Point(a.origin()[0] + along[0] * k + across[0] * sign * h, a.origin()[1] + along[1] * k + across[1] * sign * h, a.origin()[2] + along[2] * k + across[2] * sign * h));
            }
        }
        const Edge &x = ds.edges[ea], &y = ds.edges[eb];
        for (const Point& p : found) {
            const double ta = parameter(ds.curve_of(x), p, x.t0, x.t1), tb = parameter(ds.curve_of(y), p, y.t0, y.t1);
            if (ds.curve_of(x).point_at(ta).distance(p) <= tolerance && ds.curve_of(y).point_at(tb).distance(p) <= tolerance)
                r.points.push_back({p, ta, tb, false});
        }
        return true;
    }

    void ee_point(int ea, int eb, double ta, double tb) {
        const Edge& a = ds.edges[ea];
        const Edge& b = ds.edges[eb];
        const Point pa = ds.curve_of(a).point_at(ta), pb = ds.curve_of(b).point_at(tb);
        const double d = pa.distance(pb), tolerance = a.tolerance + b.tolerance + ds.fuzzy;
        if (d > tolerance)
            return;
        // A hit inside a common part is no vertex: the edges stay together on both sides of it.
        const Curve& ca = ds.curve_of(a);
        const double step = probe(ca, a, tolerance);
        if (ta - step > a.t0 && ta + step < a.t1) {
            const Point before = ca.point_at(ta - step), after = ca.point_at(ta + step);
            if (nearest(b, before).distance(before) <= tolerance && nearest(b, after).distance(after) <= tolerance)
                return;
        }
        const Point p((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2, (pa[2] + pb[2]) / 2);
        std::vector<int> candidates = edge_vertices(ea), more = edge_vertices(eb);
        candidates.insert(candidates.end(), more.begin(), more.end());
        int v = find_vertex(candidates, p, std::max(a.tolerance, b.tolerance));
        if (v < 0)
            v = ds.add_vertex(p, std::max(a.tolerance, b.tolerance) + 0.5 * d);
        add_pave(ea, v, ta);
        add_pave(eb, v, tb);
        // The common part of a crossing runs where the edges stay within the tolerance of each other: far along them at a shallow angle.
        Vector da = ca.tangent_at(ta), db = ds.curve_of(b).tangent_at(tb);
        da.normalize_self();
        db.normalize_self();
        Vertex& made = ds.vertices[ds.real(v)];
        if (made.operand < 0)
            made.reach = std::max(made.reach, std::min(tolerance / std::max(da.cross(db).magnitude(), 1e-12), extent(ca, a)));
    }

    /// IntTools_EdgeEdge within the tolerance: local minima of the distance between two edges that come within it away from a common part
    void tolerant_ee(int ea, int eb, double tolerance) {
        const Edge& a = ds.edges[ea];
        const Edge& b = ds.edges[eb];
        const Curve& ca = ds.curve_of(a);
        const Curve& cb = ds.curve_of(b);
        const int n = std::clamp(4 * ca.nurbs().cv_count() + 8, 16, 96);
        std::vector<double> d(n + 1);
        const auto distance = [&](double t) { return nearest(b, ca.point_at(t)).distance(ca.point_at(t)); };
        for (int i = 0; i <= n; ++i)
            d[i] = distance(a.t0 + (a.t1 - a.t0) * i / n);
        for (int i = 0; i <= n; ++i) {
            const bool minimum = (i == 0 || d[i] <= d[i - 1]) && (i == n || d[i] <= d[i + 1]);
            const bool run = (i > 0 && d[i - 1] <= tolerance) || (i < n && d[i + 1] <= tolerance);
            if (!minimum || run)
                continue;
            double lo = a.t0 + (a.t1 - a.t0) * std::max(i - 1, 0) / n, hi = a.t0 + (a.t1 - a.t0) * std::min(i + 1, n) / n;
            for (int k = 0; k < 60; ++k) {
                const double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
                (distance(m1) < distance(m2) ? hi : lo) = distance(m1) < distance(m2) ? m2 : m1;
            }
            const double ta = 0.5 * (lo + hi);
            if (distance(ta) <= tolerance)
                ee_point(ea, eb, ta, parameter(cb, ca.point_at(ta), b.t0, b.t1));
        }
    }

    void perform_vf() {
        for (int o = 0; o < 2; ++o)
            for (int v : real_vertices(o))
                for (int fi : act_f[1 - o]) {
                    Face& f = ds.faces[ds.face_offset[1 - o] + fi];
                    Box box;
                    box.add(at(v), tol(v));
                    if (!box.overlaps(f.box))
                        continue;
                    const std::vector<int> known = face_vertices(f);
                    if (std::find(known.begin(), known.end(), v) != known.end())
                        continue;
                    const Surface& s = ds.surface_of(f);
                    const auto [u, w] = s.closest_parameters(at(v));
                    const double d = s.point_at(u, w).distance(at(v));
                    if (d > tol(v) + f.tolerance + ds.fuzzy || f.region.state(Point(u, w, 0)) != 1)
                        continue;
                    grow(v, d + f.tolerance);
                    f.in_vertices.push_back(v);
                }
    }

    int vertex_at(int e, const Face& f, const Point& p, double tolerance) {
        std::vector<int> candidates = edge_vertices(e), more = face_vertices(f);
        candidates.insert(candidates.end(), more.begin(), more.end());
        const int v = find_vertex(candidates, p, tolerance);
        return v >= 0 ? v : ds.add_vertex(p, tolerance);
    }

    void perform_ef() {
        for (int o = 0; o < 2; ++o)
            for (int e : act_e[o])
                for (int fi : act_f[1 - o]) {
                    const int fid = ds.face_offset[1 - o] + fi;
                    Face& f = ds.faces[fid];
                    const Edge& edge = ds.edges[e];
                    if (edge.degenerated || !edge.box.overlaps(f.box))
                        continue;
                    const Curve& c = ds.curve_of(edge);
                    const Surface& s = ds.surface_of(f);
                    const double tolerance = edge.tolerance + f.tolerance + ds.fuzzy;
                    // An edge that lies on the surface from end to end has no crossing: the exact intersection of a common part is slow and tells no more.
                    int on = 0;
                    for (double k : {0.5, 0.3, 0.7, 0.1, 0.9}) {
                        const Point p = c.point_at(edge.t0 + k * (edge.t1 - edge.t0));
                        if (s.closest_point(p).distance(p) > tolerance) {
                            if (k == 0.5)
                                break;
                            continue;
                        }
                        ++on;
                    }
                    curve_surface_intersection::Result r;
                    bool exact = false;
                    try {
                        exact = on < 5 && closed_form_ef(e, fid, tolerance, r);
                    } catch (const std::exception&) {
                        r = {};
                    }
                    if (on < 5 && !exact)
                        try {
                            r = curve_surface_intersection::intersect(c.nurbs(), f.patch, tolerance);
                            // The numeric route ran on the NURBS of the edge and of the patch: back to the true parameters.
                            for (curve_surface_intersection::Hit& hit : r.points) {
                                hit.t = true_parameter(c, hit.t);
                                if (s.kind() != Surface::Kind::Nurbs)
                                    std::tie(hit.u, hit.v) = s.closest_parameters(c.point_at(hit.t));
                            }
                            for (curve_surface_intersection::Overlap& run : r.overlaps)
                                run.t0 = true_parameter(c, run.t0), run.t1 = true_parameter(c, run.t1);
                        } catch (const std::exception& error) {
                            r.complete = false;
                            r.failure = error.what();
                        }
                    if (!r.complete)
                        ds.log.push_back("EF incomplete: " + r.failure);
                    for (const curve_surface_intersection::Hit& hit : r.points)
                        ef_point(e, fid, hit.t, Point(hit.u, hit.v, 0));
                    const auto over = [&](const curve_surface_intersection::Overlap& run, bool last) {
                        if (s.kind() == Surface::Kind::Nurbs)
                            return last ? run.pcurve.point_at_end() : run.pcurve.point_at_start();
                        const auto [u, v] = s.closest_parameters(c.point_at(last ? run.t1 : run.t0));
                        return Point(u, v, 0);
                    };
                    // A run that ends inside the face leaves the surface there; one that ends on or beyond its boundary is cut by EE and VE.
                    for (const curve_surface_intersection::Overlap& run : r.overlaps)
                        for (bool last : {false, true}) {
                            const double t = last ? run.t1 : run.t0;
                            bool joined = false;
                            for (const curve_surface_intersection::Overlap& other : r.overlaps)
                                joined = joined || (&other != &run && std::min(std::abs(other.t0 - t), std::abs(other.t1 - t)) <= 1e-9 * (edge.t1 - edge.t0));
                            if (!joined && f.region.state(over(run, last)) == 1)
                                add_pave(e, vertex_at(e, f, c.point_at(t), edge.tolerance), t);
                        }
                    if (!r.complete || tolerance > 1e-6)
                        tolerant_ef(e, fid, tolerance, r);
                }
    }

    void ef_point(int e, int fid, double t, const Point& uv) {
        Face& f = ds.faces[fid];
        const int state = f.region.state(uv);
        if (state < 0)
            return;
        // A hit inside a common part of the edge and the surface is no vertex.
        const Edge& edge = ds.edges[e];
        const Curve& c = ds.curve_of(edge);
        const Surface& s = ds.surface_of(f);
        const double tolerance = edge.tolerance + f.tolerance + ds.fuzzy, step = probe(c, edge, tolerance);
        if (t - step > edge.t0 && t + step < edge.t1) {
            const Point before = c.point_at(t - step), after = c.point_at(t + step);
            if (s.closest_point(before).distance(before) <= tolerance && s.closest_point(after).distance(after) <= tolerance)
                return;
        }
        // The common part of a crossing runs where the edge stays within the tolerance of the surface.
        Vector tangent = c.tangent_at(t), normal = s.normal_at(uv[0], uv[1]);
        tangent.normalize_self();
        normal.normalize_self();
        const Point p = c.point_at(t);
        const double reach = std::min(tolerance / std::max(std::abs(tangent.dot(normal)), 1e-12), extent(c, edge));
        // PerformEF, a common part that reaches a pave of the edge: no new vertex, the pave's vertex grows to cover a real crossing.
        if (s.point_at(uv[0], uv[1]).distance(p) < confusion)
            for (const Pave& pave : edge.paves) {
                const double d = at(pave.vertex).distance(p), limit = tol(pave.vertex) < 0.01 ? std::min(1e4 * tol(pave.vertex), 0.1) : 1e4 * tol(pave.vertex);
                if (d - reach > std::max(tol(pave.vertex), ds.vertices[ds.real(pave.vertex)].reach) || d >= limit)
                    continue;
                grow(pave.vertex, d);
                const std::vector<int> known = face_vertices(f);
                if (state == 1 && std::find(known.begin(), known.end(), ds.real(pave.vertex)) == known.end())
                    f.in_vertices.push_back(ds.real(pave.vertex));
                return;
            }
        const int v = vertex_at(e, f, p, edge.tolerance);
        add_pave(e, v, t);
        Vertex& made = ds.vertices[ds.real(v)];
        if (made.operand < 0)
            made.reach = std::max(made.reach, reach);
        if (state == 1 && std::find(f.in_vertices.begin(), f.in_vertices.end(), v) == f.in_vertices.end())
            f.in_vertices.push_back(v);
    }

    /// IntTools_EdgeFace within the tolerance: runs of an edge lying on the face and isolated touches the exact intersection does not see
    void tolerant_ef(int e, int fid, double tolerance, const curve_surface_intersection::Result& exact) {
        const Edge& edge = ds.edges[e];
        const Face& f = ds.faces[fid];
        const Curve& c = ds.curve_of(edge);
        const Surface& s = ds.surface_of(f);
        const int n = std::clamp(4 * c.nurbs().cv_count() + 8, 16, 96);
        std::vector<double> t(n + 1), d(n + 1);
        std::vector<int> state(n + 1);
        const auto measure = [&](double at, double& distance, Point& uv) {
            const Point p = c.point_at(at);
            const auto [u, v] = s.closest_parameters(p);
            uv = Point(u, v, 0);
            distance = s.point_at(u, v).distance(p);
        };
        for (int i = 0; i <= n; ++i) {
            t[i] = edge.t0 + (edge.t1 - edge.t0) * i / n;
            Point uv;
            measure(t[i], d[i], uv);
            state[i] = d[i] <= tolerance ? f.region.state(uv) : -1;
        }
        const auto covered = [&](double at) {
            for (const auto& overlap : exact.overlaps)
                if (at >= std::min(overlap.t0, overlap.t1) - 1e-9 && at <= std::max(overlap.t0, overlap.t1) + 1e-9)
                    return true;
            return false;
        };
        // The parameter where the state changes between a sample on the face and one off it.
        const auto edge_of_run = [&](double on, double off) {
            for (int k = 0; k < 40; ++k) {
                const double mid = 0.5 * (on + off);
                double distance;
                Point uv;
                measure(mid, distance, uv);
                (distance <= tolerance && f.region.state(uv) >= 0 ? on : off) = mid;
            }
            return on;
        };
        for (int i = 0; i <= n;) {
            if (state[i] < 0) {
                ++i;
                continue;
            }
            int j = i;
            while (j < n && state[j + 1] >= 0)
                ++j;
            if (j > i) {
                // A run over the whole edge is an In block; a short one is a crossing; a long one ends where the edge leaves the surface inside the face.
                const double a = i > 0 ? edge_of_run(t[i], t[i - 1]) : t[i], b = j < n ? edge_of_run(t[j], t[j + 1]) : t[j];
                bool crossed = covered(0.5 * (a + b));
                for (const curve_surface_intersection::Hit& hit : exact.points)
                    crossed = crossed || (hit.t >= a && hit.t <= b);
                if (crossed || (i == 0 && j == n)) {
                } else if (c.point_at(a).distance(c.point_at(b)) <= 8.0 * tolerance) {
                    double lo = a, hi = b, distance;
                    Point uv;
                    for (int k = 0; k < 60; ++k) {
                        const double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
                        double d1, d2;
                        measure(m1, d1, uv);
                        measure(m2, d2, uv);
                        (d1 < d2 ? hi : lo) = d1 < d2 ? m2 : m1;
                    }
                    measure(0.5 * (lo + hi), distance, uv);
                    ef_point(e, fid, 0.5 * (lo + hi), uv);
                } else
                    for (const auto& [end, inner, step] : {std::tuple{a, i > 0, -1e-3}, std::tuple{b, j < n, 1e-3}}) {
                        // Just past the end the edge is off the surface yet still over the face: it left the surface, not the face.
                        double distance;
                        Point uv;
                        measure(std::clamp(end + step * (edge.t1 - edge.t0), edge.t0, edge.t1), distance, uv);
                        if (inner && distance > tolerance && f.region.state(uv) == 1)
                            add_pave(e, vertex_at(e, f, c.point_at(end), edge.tolerance), end);
                    }
            } else if (!covered(t[i])) {
                double lo = i > 0 ? t[i - 1] : t[i], hi = i < n ? t[i + 1] : t[i];
                for (int k = 0; k < 60; ++k) {
                    const double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
                    double d1, d2;
                    Point uv;
                    measure(m1, d1, uv);
                    measure(m2, d2, uv);
                    (d1 < d2 ? hi : lo) = d1 < d2 ? m2 : m1;
                }
                double distance;
                Point uv;
                measure(0.5 * (lo + hi), distance, uv);
                if (distance <= tolerance)
                    ef_point(e, fid, 0.5 * (lo + hi), uv);
            }
            i = j + 1;
        }
    }

    /// Sort the paves of every edge, merge paves whose vertices touch, and cut the edge into pave blocks
    void make_blocks() {
        for (int o = 0; o < 2; ++o)
            for (int e : act_e[o]) {
                Edge& edge = ds.edges[e];
                for (Pave& p : edge.paves)
                    p.vertex = ds.real(p.vertex);
                Pave first = edge.paves[0], last = edge.paves[1];
                std::vector<Pave> inner(edge.paves.begin() + 2, edge.paves.end());
                std::sort(inner.begin(), inner.end(), [](const Pave& x, const Pave& y) { return x.t < y.t; });
                std::vector<Pave> paves{first};
                for (const Pave& p : inner) {
                    if (ds.real(p.vertex) == ds.real(paves.back().vertex))
                        continue;
                    if (!edge.degenerated && at(p.vertex).distance(at(paves.back().vertex)) <= tol(p.vertex) + tol(paves.back().vertex)) {
                        ds.merge({p.vertex, paves.back().vertex});
                        continue;
                    }
                    paves.push_back(p);
                }
                while (paves.size() > 1 && (ds.real(paves.back().vertex) == ds.real(last.vertex)
                    || at(paves.back().vertex).distance(at(last.vertex)) <= tol(paves.back().vertex) + tol(last.vertex))) {
                    if (ds.real(paves.back().vertex) != ds.real(last.vertex))
                        ds.merge({paves.back().vertex, last.vertex});
                    paves.pop_back();
                }
                paves.push_back(last);
                edge.paves = paves;
                edge.blocks.clear();
                for (size_t i = 0; i + 1 < paves.size(); ++i) {
                    edge.blocks.push_back(static_cast<int>(ds.blocks.size()));
                    ds.blocks.push_back({e, -1, paves[i], paves[i + 1]});
                }
            }
        for (PaveBlock& b : ds.blocks) {
            b.first.vertex = ds.real(b.first.vertex);
            b.last.vertex = ds.real(b.last.vertex);
        }
    }

    std::pair<int, int> key(const PaveBlock& b) const {
        const int v = ds.real(b.first.vertex), w = ds.real(b.last.vertex);
        return {std::min(v, w), std::max(v, w)};
    }

    /// Whether a point lies on a block within the tolerance, strictly between its paves
    bool on_block(const PaveBlock& b, const Point& p, double tolerance) const {
        if (b.edge >= 0 && ds.edges[b.edge].degenerated)
            return false;
        const Curve& c = ds.block_curve(b);
        const double lo = std::min(b.first.t, b.last.t), hi = std::max(b.first.t, b.last.t);
        if (c.kind() != Curve::Kind::Nurbs)
            return c.point_at(parameter(c, p, lo, hi)).distance(p) <= tolerance;
        double t = c.closest_parameter(p);
        if (t < lo || t > hi) {
            // A closest point outside the range may still have a near twin inside it on a closed curve.
            NurbsCurve piece = c.nurbs();
            if (!piece.trim(lo, hi))
                return false;
            t = piece.closest_parameter(p);
            return piece.point_at(t).distance(p) <= tolerance;
        }
        return c.point_at(t).distance(p) <= tolerance;
    }

    bool coincide(const PaveBlock& a, const PaveBlock& b, double tolerance) const {
        if (key(a) != key(b))
            return false;
        for (double s : {0.25, 0.5, 0.75})
            if (!on_block(b, ds.block_point(a, s), tolerance) || !on_block(a, ds.block_point(b, s), tolerance))
                return false;
        return true;
    }

    void join(int a, int b) {
        int keep = ds.blocks[a].common >= 0 ? ds.blocks[a].common : ds.blocks[b].common;
        if (keep < 0) {
            keep = static_cast<int>(ds.commons.size());
            ds.commons.push_back({});
        }
        for (int k : {a, b}) {
            const int c = ds.blocks[k].common;
            if (c == keep)
                continue;
            for (int x : c >= 0 ? ds.commons[c].blocks : std::vector<int>{k}) {
                ds.blocks[x].common = keep;
                ds.commons[keep].blocks.push_back(x);
            }
            if (c >= 0)
                ds.commons[c].blocks.clear();
        }
    }

    /// Common blocks of edges lying on each other (the EE common parts of PerformEE)
    void make_commons() {
        std::map<std::pair<int, int>, std::vector<int>> by_key;
        for (int o = 0; o < 2; ++o)
            for (int e : act_e[o])
                for (int b : ds.edges[e].blocks)
                    if (!ds.edges[e].degenerated)
                        by_key[key(ds.blocks[b])].push_back(b);
        for (const auto& [unused, list] : by_key)
            for (size_t i = 0; i < list.size(); ++i)
                for (size_t j = i + 1; j < list.size(); ++j) {
                    const PaveBlock &a = ds.blocks[list[i]], &b = ds.blocks[list[j]];
                    if (ds.edges[a.edge].operand == ds.edges[b.edge].operand)
                        continue;
                    if (coincide(a, b, ds.edges[a.edge].tolerance + ds.edges[b.edge].tolerance + ds.fuzzy))
                        join(list[i], list[j]);
                }
    }

    /// Whether the image of a block is already a boundary, In or Sc edge of the face
    bool face_has(const Face& f, int b) const {
        for (int k : ds.family(b)) {
            const PaveBlock& x = ds.blocks[k];
            if (x.edge >= 0 && std::find(f.edges.begin(), f.edges.end(), x.edge) != f.edges.end())
                return true;
            for (const std::vector<int>* list : {&f.in_blocks, &f.sc_blocks})
                for (int y : *list)
                    if (ds.blocks[y].common >= 0 ? ds.blocks[y].common == x.common : y == k)
                        return true;
        }
        return false;
    }

    /// In blocks (IntTools_EdgeFace::IsCoincident as ForceInterfEF calls it): a block between two vertices of a face of the other operand that runs on it
    void make_in_blocks() {
        for (int o = 0; o < 2; ++o)
            for (int fi : act_f[1 - o]) {
                const int fid = ds.face_offset[1 - o] + fi;
                Face& f = ds.faces[fid];
                const Surface& s = ds.surface_of(f);
                std::vector<int> known = face_vertices(f);
                std::sort(known.begin(), known.end());
                const bool flat = s.kind() == Surface::Kind::Plane || (s.kind() == Surface::Kind::Nurbs && s.nurbs().degree(0) == 1 && s.nurbs().degree(1) == 1);
                for (int e : act_e[o]) {
                    const Edge& edge = ds.edges[e];
                    if (edge.degenerated || !edge.box.overlaps(f.box))
                        continue;
                    const Curve& c = ds.curve_of(edge);
                    for (int b : edge.blocks) {
                        const PaveBlock& block = ds.blocks[b];
                        if (!std::binary_search(known.begin(), known.end(), block.first.vertex) || !std::binary_search(known.begin(), known.end(), block.last.vertex) || face_has(f, b))
                            continue;
                        const auto gap = [&](double t, Point& uv) {
                            const Point p = c.point_at(t);
                            const auto [u, v] = s.closest_parameters(p);
                            uv = Point(u, v, 0);
                            return s.point_at(u, v).distance(p);
                        };
                        // The ends may sit off the face by up to their vertices' tolerance: that much is added to the criterion.
                        const double check = 2.0 * std::max(tol(block.first.vertex), tol(block.last.vertex)), span = block.last.t - block.first.t;
                        double criterion = edge.tolerance + f.tolerance + ds.fuzzy;
                        Point uv;
                        for (double t : {block.first.t + 0.01 * span, block.last.t - 0.01 * span}) {
                            const double d = gap(t, uv);
                            if (d < check)
                                criterion = std::max(criterion, d + ds.fuzzy);
                        }
                        const int n = flat && straight(c) ? 2 : 8;
                        int on = 0;
                        bool out = false;
                        for (int i = 0; i <= n && !out; ++i) {
                            const double d = gap(block.first.t + (0.01 + 0.98 * i / n) * span, uv);
                            out = d > 100.0 * criterion || (i == n / 2 && (d > criterion || f.region.state(uv) != 1)) || (d <= criterion && f.region.state(uv) < 0);
                            on += d <= criterion;
                        }
                        if (out || 2 * on <= n + 1)
                            continue;
                        f.in_blocks.push_back(b);
                        ds.pcurves[{b, fid}] = owned(project(c, block.first.t, block.last.t, s));
                    }
                }
            }
    }

    std::vector<int> face_blocks(const Face& f) const {
        std::vector<int> result = f.in_blocks;
        result.insert(result.end(), f.sc_blocks.begin(), f.sc_blocks.end());
        for (int e : f.edges)
            result.insert(result.end(), ds.edges[e].blocks.begin(), ds.edges[e].blocks.end());
        return result;
    }

    void perform_ff() {
        for (int fa : act_f[0])
            for (int fb : act_f[1]) {
                const int ia = ds.face_offset[0] + fa, ib = ds.face_offset[1] + fb;
                if (!ds.faces[ia].box.overlaps(ds.faces[ib].box))
                    continue;
                bop_ff::Result r;
                try {
                    r = bop_ff::intersect(ds.brep(0), fa, ds.brep(1), fb, ds.fuzzy);
                } catch (const std::exception& error) {
                    ds.log.push_back(std::string("FF ") + error.what());
                    continue;
                }
                if (r.same_domain) {
                    ds.same_domain_faces.push_back({ia, ib});
                    same_domain_in_blocks(ia, ib);
                    same_domain_in_blocks(ib, ia);
                }
                const std::vector<int> loop = loops(r.curves);
                std::map<int, Point> origins;
                for (int chain : loop) {
                    Point origin;
                    if (chain >= 0 && !origins.count(chain) && !crosses_seam(ia, ib, r.curves, loop, chain) && loop_origin(ia, ib, r.curves, loop, chain, origin))
                        origins[chain] = origin;
                }
                const size_t first = ds.curves.size();
                for (size_t i = 0; i < r.curves.size(); ++i)
                    make_section(ia, ib, r.curves[i], origins.count(loop[i]) ? &origins[loop[i]] : nullptr);
                missed_sections(ia, ib, first);
                // Another closed curve starts at a vertex it already has (a seam crossing); one without any keeps its start.
                for (size_t i = 0; i < loop.size(); ++i) {
                    if (loop[i] != static_cast<int>(i) || origins.count(loop[i]))
                        continue;
                    bool kept = false;
                    for (size_t k = i; k < loop.size(); ++k)
                        for (const Pave& p : ds.curves[first + k].paves)
                            kept = kept || (loop[k] == loop[i] && !ds.vertices[ds.real(p.vertex)].joint);
                    if (!kept && !ds.curves[first + i].paves.empty())
                        ds.vertices[ds.real(ds.curves[first + i].paves[0].vertex)].joint = false;
                }
            }
    }

    /// A recorded check: a vertex both faces hold, where an edge of one crosses the other, lies on a section of the pair or on an edge they share
    void missed_sections(int ia, int ib, size_t first) {
        std::vector<int> va = face_vertices(ds.faces[ia]), vb = face_vertices(ds.faces[ib]), both, used;
        for (std::vector<int>* list : {&va, &vb}) {
            std::sort(list->begin(), list->end());
            list->erase(std::unique(list->begin(), list->end()), list->end());
        }
        std::set_intersection(va.begin(), va.end(), vb.begin(), vb.end(), std::back_inserter(both));
        for (size_t k = first; k < ds.curves.size(); ++k)
            for (const Pave& p : ds.curves[k].paves)
                used.push_back(ds.real(p.vertex));
        for (int b : face_blocks(ds.faces[ia]))
            if (face_has(ds.faces[ib], b))
                used.insert(used.end(), {ds.real(ds.blocks[b].first.vertex), ds.real(ds.blocks[b].last.vertex)});
        // A vertex on a boundary edge of each face is where two edges cross: no section of the pair has to pass there.
        const auto on_boundary = [&](int f, int v) {
            for (int e : ds.faces[f].edges)
                if (has_pave(e, v))
                    return true;
            return false;
        };
        int missed = 0;
        for (int v : both)
            missed += ds.vertices[v].reach > 0.0 && std::find(used.begin(), used.end(), v) == used.end() && !(on_boundary(ia, v) && on_boundary(ib, v));
        if (missed > 0)
            ds.log.push_back("FF incomplete: " + std::to_string(missed) + " edge crossings on no section");
    }

    /// The edges of a face lying inside a face on the same surface are In edges of it, whatever EF found
    void same_domain_in_blocks(int target, int source) {
        Face& f = ds.faces[target];
        const Surface& s = ds.surface_of(f);
        for (int e : ds.faces[source].edges) {
            const Edge& edge = ds.edges[e];
            if (edge.degenerated)
                continue;
            for (int b : edge.blocks) {
                if (face_has(f, b))
                    continue;
                const PaveBlock& block = ds.blocks[b];
                const Point mid = ds.block_point(block, 0.5);
                const auto [u, v] = s.closest_parameters(mid);
                if (s.point_at(u, v).distance(mid) > edge.tolerance + f.tolerance + ds.fuzzy || f.region.state(Point(u, v, 0)) != 1)
                    continue;
                f.in_blocks.push_back(b);
                ds.pcurves[{b, target}] = owned(project(ds.curve_of(edge), block.first.t, block.last.t, s));
            }
        }
    }

    /// Closed chains of section pieces: per piece the first piece of its chain, -1 for a piece of an open one
    std::vector<int> loops(const std::vector<bop_ff::Curve>& curves) const {
        const size_t n = curves.size();
        std::vector<Point> ends;
        std::vector<double> reach;
        for (const bop_ff::Curve& c : curves)
            for (bool last : {false, true}) {
                ends.push_back(c.curve.point_at(last ? c.curve.domain().second : c.curve.domain().first));
                reach.push_back(std::max(c.tolerance, confusion));
            }
        std::vector<int> node(2 * n), piece(n);
        std::iota(node.begin(), node.end(), 0);
        std::iota(piece.begin(), piece.end(), 0);
        const auto root = [](std::vector<int>& parent, int k) {
            while (parent[k] != k)
                k = parent[k] = parent[parent[k]];
            return k;
        };
        for (size_t i = 0; i < 2 * n; ++i)
            for (size_t j = i + 1; j < 2 * n; ++j)
                if (ends[i].distance(ends[j]) <= reach[i] + reach[j] + ds.fuzzy) {
                    const int a = root(node, static_cast<int>(i)), b = root(node, static_cast<int>(j)), x = root(piece, static_cast<int>(i / 2)), y = root(piece, static_cast<int>(j / 2));
                    node[std::max(a, b)] = std::min(a, b);
                    piece[std::max(x, y)] = std::min(x, y);
                }
        std::vector<int> degree(2 * n, 0), result(n);
        for (size_t i = 0; i < 2 * n; ++i)
            ++degree[root(node, static_cast<int>(i))];
        std::vector<bool> open(n, false);
        for (size_t i = 0; i < 2 * n; ++i)
            if (degree[root(node, static_cast<int>(i))] != 2)
                open[root(piece, static_cast<int>(i / 2))] = true;
        for (size_t i = 0; i < n; ++i)
            result[i] = open[root(piece, static_cast<int>(i))] ? -1 : root(piece, static_cast<int>(i));
        return result;
    }

    /// Where IntAna_QuadQuadGeo starts the circle or ellipse a plane cuts from a sphere, cylinder or cone (DirToAx2); false for another closed curve, which OCCT starts at a seam
    bool loop_origin(int ia, int ib, const std::vector<bop_ff::Curve>& curves, const std::vector<int>& loop, int chain, Point& origin) const {
        const Point fallback = curves[chain].curve.point_at(curves[chain].curve.domain().first);
        for (const auto& [flat, other] : {std::pair{ia, ib}, std::pair{ib, ia}}) {
            // The plane and the quadric as the surfaces say them; recognized from the patches while those are freeform.
            const Surface& plane = ds.face_surface(ds.faces[flat]);
            const Surface& round = ds.face_surface(ds.faces[other]);
            Point o = plane.origin();
            Vector a = plane.x(), b = plane.y();
            if (plane.kind() != Surface::Kind::Plane && !affine_patch(plane.nurbs(), o, a, b))
                continue;
            const NurbsSurface& s = round.nurbs();
            nurbs_implicit::Implicit form;
            const bool said = round.kind() == Surface::Kind::Sphere || round.kind() == Surface::Kind::Cylinder || round.kind() == Surface::Kind::Cone;
            if (said) {
                form.kind = round.kind() == Surface::Kind::Sphere ? 2 : round.kind() == Surface::Kind::Cylinder ? 3 : 4;
                form.point = {round.origin()[0], round.origin()[1], round.origin()[2]};
                form.axis = {round.z()[0], round.z()[1], round.z()[2]};
                form.radius = round.radius();
            } else
                try {
                    form = nurbs_implicit::recognize(s, std::max(ds.faces[other].tolerance, confusion));
                } catch (const std::exception&) {
                    continue;
                }
            Vector n = plane.kind() == Surface::Kind::Plane ? plane.z() : a.cross(b), axis(form.axis[0], form.axis[1], form.axis[2]);
            if (!said && (form.kind == 3 || form.kind == 4)) {
                const int d = s.degree(0) == 1 ? 0 : 1;
                const double u = 0.5 * (s.domain(1 - d).first + s.domain(1 - d).second);
                const Point p0 = d == 0 ? s.point_at(s.domain(0).first, u) : s.point_at(u, s.domain(1).first), p1 = d == 0 ? s.point_at(s.domain(0).second, u) : s.point_at(u, s.domain(1).second);
                if (axis.dot(Vector(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])) < 0)
                    axis *= -1.0;
            }
            if (!conic_origin(o, n, form.kind, Point(form.point[0], form.point[1], form.point[2]), axis, form.radius, fallback, origin))
                continue;
            for (size_t i = 0; i < curves.size(); ++i)
                if (loop[i] == chain && curves[i].curve.point_at(parameter(curves[i].curve, origin, curves[i].curve.domain().first, curves[i].curve.domain().second)).distance(origin) <= std::max(curves[i].tolerance, confusion) + ds.fuzzy)
                    return true;
        }
        return false;
    }

    /// Whether a closed chain crosses a seam of either face: IntPatch starts the curve there, inside the other face or not
    bool crosses_seam(int ia, int ib, const std::vector<bop_ff::Curve>& curves, const std::vector<int>& loop, int chain) const {
        for (int f : {ia, ib})
            for (int e : ds.faces[f].seams)
                for (size_t i = 0; i < curves.size(); ++i) {
                    if (loop[i] != chain || ds.edges[e].degenerated)
                        continue;
                    // A section cut at the seam ends on it: the intersection of two curves may miss a hit at an end.
                    const double reach = ds.edges[e].tolerance + curves[i].tolerance + ds.fuzzy;
                    for (double t : {curves[i].curve.domain().first, curves[i].curve.domain().second})
                        if (nearest(ds.edges[e], curves[i].curve.point_at(t)).distance(curves[i].curve.point_at(t)) <= reach)
                            return true;
                    try {
                        if (!curve_intersection::intersect(ds.curve_of(ds.edges[e]).nurbs(), curves[i].curve.nurbs(), ds.edges[e].tolerance + curves[i].tolerance + ds.fuzzy).points.empty())
                            return true;
                    } catch (const std::exception&) {
                    }
                }
        return false;
    }



    void make_section(int ia, int ib, bop_ff::Curve& section, const Point* origin) {
        Section c{ia, ib, std::make_shared<const Curve>(section.curve), std::make_shared<const Curve>(section.pcurve_a), std::make_shared<const Curve>(section.pcurve_b),
            std::max(section.tolerance, confusion), {}};
        const Curve& w = *c.world;
        const auto [t0, t1] = w.domain();
        const bool closed = w.point_at(t0).distance(w.point_at(t1)) <= c.tolerance;
        std::vector<int> candidates;
        for (int f : {ia, ib}) {
            std::vector<int> more = face_vertices(ds.faces[f]);
            candidates.insert(candidates.end(), more.begin(), more.end());
            for (int b : face_blocks(ds.faces[f]))
                candidates.insert(candidates.end(), {ds.real(ds.blocks[b].first.vertex), ds.real(ds.blocks[b].last.vertex)});
        }
        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
        Box box = curve_box(w.nurbs(), c.tolerance);
        for (int v : candidates) {
            Box ball;
            ball.add(at(v), std::max(tol(v), ds.vertices[ds.real(v)].reach) + ds.fuzzy);
            if (!ball.overlaps(box))
                continue;
            const double t = parameter(w, at(v), t0, t1);
            const double d = w.point_at(t).distance(at(v));
            if (d > std::max(tol(v), ds.vertices[ds.real(v)].reach) + c.tolerance + ds.fuzzy)
                continue;
            grow(v, d);
            // A vertex over the bound of a closed curve is the pave of both its ends.
            if (closed && at(v).distance(w.point_at(t0)) <= tol(v) + c.tolerance)
                c.paves.insert(c.paves.end(), {{v, t0}, {v, t1}});
            else
                c.paves.push_back({v, t});
        }
        const auto near = [&](double t) {
            for (const Pave& p : c.paves)
                if (at(p.vertex).distance(w.point_at(t)) <= std::max(tol(p.vertex), ds.vertices[ds.real(p.vertex)].reach) + c.tolerance)
                    return true;
            return false;
        };
        if (!near(t0)) {
            const int v = ds.add_vertex(w.point_at(t0), c.tolerance);
            ds.vertices[v].joint = !section.bound[0];
            c.paves.push_back({v, t0});
            if (closed)
                c.paves.push_back({v, t1});
        }
        if (!near(t1)) {
            c.paves.push_back({ds.add_vertex(w.point_at(t1), c.tolerance), t1});
            ds.vertices.back().joint = !section.bound[1];
        }
        // PutBoundPaveOnCurve: a closed curve keeps a vertex at its bound unless one sits there already.
        const double bound = origin ? parameter(w, *origin, t0, t1) : t0;
        if (origin && w.point_at(bound).distance(*origin) <= c.tolerance + confusion + ds.fuzzy) {
            bool found = false;
            for (const Pave& p : c.paves)
                if (at(p.vertex).distance(w.point_at(bound)) <= tol(p.vertex) + c.tolerance) {
                    ds.vertices[ds.real(p.vertex)].joint = false;
                    found = true;
                }
            if (!found)
                c.paves.push_back({ds.add_vertex(w.point_at(bound), c.tolerance), bound});
        }
        std::sort(c.paves.begin(), c.paves.end(), [](const Pave& x, const Pave& y) { return x.t < y.t; });
        std::vector<Pave> paves;
        for (const Pave& p : c.paves) {
            if (!paves.empty() && ds.real(p.vertex) == ds.real(paves.back().vertex) && p.t - paves.back().t < 1e-9 * (t1 - t0))
                continue;
            paves.push_back({ds.real(p.vertex), p.t});
        }
        c.paves = paves;
        const int curve = static_cast<int>(ds.curves.size());
        ds.curves.push_back(c);
        for (size_t i = 0; i + 1 < paves.size(); ++i) {
            PaveBlock block{-1, curve, {ds.real(paves[i].vertex), paves[i].t}, {ds.real(paves[i + 1].vertex), paves[i + 1].t}};
            const Point mid = w.point_at(0.5 * (block.first.t + block.last.t));
            // BRepLib::FindValidRange: a piece inside the tolerance spheres of its vertices is no edge, its vertices are one.
            bool covered = true;
            for (double k : {0.25, 0.5, 0.75}) {
                const Point p = w.point_at(block.first.t + k * (block.last.t - block.first.t));
                covered = covered && (p.distance(at(block.first.vertex)) <= std::max(tol(block.first.vertex), c.tolerance) || p.distance(at(block.last.vertex)) <= std::max(tol(block.last.vertex), c.tolerance));
            }
            if (covered) {
                ds.merge({block.first.vertex, block.last.vertex});
                continue;
            }
            const Section& cc = ds.curves[curve];
            const Span pa = ranged(cc.pcurve_a, {t0, t1}, block.first.t, block.last.t), pb = ranged(cc.pcurve_b, {t0, t1}, block.first.t, block.last.t);
            if (ds.faces[ia].region.state(pa.at(0.5)) < 0 || ds.faces[ib].region.state(pb.at(0.5)) < 0)
                continue;
            // IsExistingPaveBlock: a piece whose ends and middle lie on a block of either face is that block.
            int existing = -1;
            for (int f : {ia, ib})
                for (int b : face_blocks(ds.faces[f])) {
                    const PaveBlock& old = ds.blocks[b];
                    // A block shared with a face or another edge may lie as far off as its vertices reach, twice that when both faces hold it; one between the same vertices running the same way, up to ten times the curve's tolerance.
                    const double fat = std::max({tol(block.first.vertex), tol(block.last.vertex), tol(old.first.vertex), tol(old.last.vertex)}) + ds.fuzzy;
                    double reach = ds.block_tolerance(old) + c.tolerance + ds.fuzzy;
                    bool shared = old.common >= 0;
                    for (const Face& g : ds.faces)
                        shared = shared || std::find(g.in_blocks.begin(), g.in_blocks.end(), b) != g.in_blocks.end();
                    if (shared)
                        reach = std::max(reach, fat) * (face_has(ds.faces[ia], b) && face_has(ds.faces[ib], b) ? 2.0 : 1.0);
                    else if (key(old) == key(block) && (!straight(w) || !straight(ds.block_curve(old)))) {
                        const Curve& oc = ds.block_curve(old);
                        Vector t1 = w.tangent_at(0.5 * (block.first.t + block.last.t)), t2 = oc.tangent_at(parameter(oc, mid, oc.domain().first, oc.domain().second));
                        t1.normalize_self();
                        t2.normalize_self();
                        if (std::abs(t1.dot(t2)) >= 0.9063)
                            reach = std::max(reach, 2.0 * std::min({1e-3, 10.0 * (c.tolerance + ds.fuzzy), std::max(c.tolerance + ds.fuzzy, fat)}));
                    }
                    bool on = existing < 0;
                    for (int v : {block.first.vertex, block.last.vertex})
                        on = on && (v == old.first.vertex || v == old.last.vertex || on_block(old, at(v), reach + tol(v)));
                    // HasSameBounds: the block ends where the piece ends, not somewhere along it.
                    for (int v : {old.first.vertex, old.last.vertex})
                        on = on && (v == block.first.vertex || v == block.last.vertex || std::min(at(v).distance(at(block.first.vertex)), at(v).distance(at(block.last.vertex))) <= reach + tol(v) + fat);
                    if (on && on_block(old, mid, reach))
                        existing = b;
                }
            if (existing < 0) {
                existing = static_cast<int>(ds.blocks.size());
                ds.blocks.push_back(block);
            }
            // An existing edge that runs off the section keeps its own curve: its pcurve is its projection, as BuildPCurveForEdgeOnFace makes it.
            const PaveBlock& kept = ds.blocks[existing];
            const bool same = key(kept) == key(block) && on_block(kept, mid, 2.0 * (c.tolerance + ds.fuzzy)), along = same && kept.first.vertex == block.first.vertex;
            for (int f : {ia, ib})
                if (!face_has(ds.faces[f], existing)) {
                    ds.faces[f].sc_blocks.push_back(existing);
                    const Span& p = f == ia ? pa : pb;
                    ds.pcurves[{existing, f}] = !same ? owned(project(ds.block_curve(kept), kept.first.t, kept.last.t, ds.surface_of(ds.faces[f]))) : along ? p : p.reversed();
                }
        }
    }
};

}

DS fill(const BRep& a, const BRep& b, double fuzzy) {
    DS ds;
    ds.fuzzy = fuzzy;
    Filler filler{ds, {}, {}, {}, {}, {}};
    filler.init(a, b);
    filler.perform_vv();
    filler.perform_ve();
    filler.perform_ee();
    filler.perform_vf();
    filler.perform_ef();
    filler.make_blocks();
    filler.make_commons();
    filler.make_in_blocks();
    filler.perform_ff();
    for (PaveBlock& b : ds.blocks) {
        b.first.vertex = ds.real(b.first.vertex);
        b.last.vertex = ds.real(b.last.vertex);
    }
    return ds;
}

}
