#include "bop_ds.h"
#include "bop_ff.h"
#include "curve_intersection.h"
#include "curve_surface_intersection.h"
#include "nurbs_implicit.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>

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

    double tol(int v) const { return ds.vertices[ds.real(v)].tolerance; }
    const Point& at(int v) const { return ds.vertices[ds.real(v)].point; }

    /// How far an edge's curve runs from a face it bounds: the edge tolerance of BRepLib::SameParameter, which loose files do not carry
    static double deviation(const BRep& brep, const BRepRef& use, int face) {
        const BRepEdge& e = brep.m_edges[use.index];
        const int index = brep.pcurve_index(use.index, face, use.orientation);
        if (e.degenerated || index < 0)
            return 0.0;
        const NurbsCurve& c = brep.m_curves_3d[e.curve_3d_index];
        const NurbsCurve& p = brep.m_curves_2d[index];
        const NurbsSurface& s = brep.m_surfaces[brep.m_faces[face].surface_index];
        double worst = 0.0;
        for (int i = 1; i < 8; ++i) {
            const Point q = c.point_at(c.domain_start() + i / 8.0 * (c.domain_end() - c.domain_start()));
            const Point uv = p.point_at(p.domain_start() + i / 8.0 * (p.domain_end() - p.domain_start()));
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
                if (e.degenerated) {
                    const NurbsCurve& p = brep.m_curves_2d[e.pcurves.at(0).curve_2d_index];
                    edge.t0 = p.domain_start();
                    edge.t1 = p.domain_end();
                } else {
                    const NurbsCurve& c = brep.m_curves_3d[e.curve_3d_index];
                    edge.t0 = c.domain_start();
                    edge.t1 = c.domain_end();
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
                        ds.edges[e].tolerance = std::max({ds.edges[e].tolerance, face.tolerance, deviation(brep, use, f)});
                        used_e[use.index] = true;
                    }
                face.region = face_region(brep, f, face.tolerance);
                face.box = surface_box(ds.surface_of(face), face.tolerance);
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
                    edge.box = curve_box(ds.curve_of(edge), edge.tolerance);
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
    static double probe(const NurbsCurve& c, const Edge& edge, double tolerance) {
        const double length = c.point_at(edge.t0).distance(c.point_at(edge.t1)) + c.point_at(edge.t0).distance(c.point_at(0.5 * (edge.t0 + edge.t1)));
        return (edge.t1 - edge.t0) * std::max(1e-2, 4.0 * tolerance / std::max(length, 1e-300));
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
                    const NurbsCurve& c = ds.curve_of(edge);
                    const double t = std::clamp(c.closest_parameter(at(v)), edge.t0, edge.t1);
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
                    r = curve_intersection::intersect(ds.curve_of(a), ds.curve_of(b), tolerance);
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

    void ee_point(int ea, int eb, double ta, double tb) {
        const Edge& a = ds.edges[ea];
        const Edge& b = ds.edges[eb];
        const Point pa = ds.curve_of(a).point_at(ta), pb = ds.curve_of(b).point_at(tb);
        const double d = pa.distance(pb), tolerance = a.tolerance + b.tolerance + ds.fuzzy;
        if (d > tolerance)
            return;
        // A hit inside a common part is no vertex: the edges stay together on both sides of it.
        const NurbsCurve& ca = ds.curve_of(a);
        const double step = probe(ca, a, tolerance);
        if (ta - step > a.t0 && ta + step < a.t1) {
            const Point before = ca.point_at(ta - step), after = ca.point_at(ta + step);
            if (ds.curve_of(b).closest_point(before).distance(before) <= tolerance && ds.curve_of(b).closest_point(after).distance(after) <= tolerance)
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
    }

    /// IntTools_EdgeEdge within the tolerance: local minima of the distance between two edges that come within it away from a common part
    void tolerant_ee(int ea, int eb, double tolerance) {
        const Edge& a = ds.edges[ea];
        const NurbsCurve& ca = ds.curve_of(a);
        const NurbsCurve& cb = ds.curve_of(ds.edges[eb]);
        const int n = std::clamp(4 * ca.cv_count() + 8, 16, 96);
        std::vector<double> d(n + 1);
        const auto distance = [&](double t) { return cb.closest_point(ca.point_at(t)).distance(ca.point_at(t)); };
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
                ee_point(ea, eb, ta, cb.closest_parameter(ca.point_at(ta)));
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
                    const NurbsSurface& s = ds.surface_of(f);
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
                    const NurbsCurve& c = ds.curve_of(edge);
                    const NurbsSurface& s = ds.surface_of(f);
                    const double tolerance = edge.tolerance + f.tolerance + ds.fuzzy;
                    curve_surface_intersection::Result r;
                    try {
                        r = curve_surface_intersection::intersect(c, s, tolerance);
                    } catch (const std::exception& error) {
                        r.complete = false;
                        r.failure = error.what();
                    }
                    if (!r.complete)
                        ds.log.push_back("EF incomplete: " + r.failure);
                    for (const curve_surface_intersection::Hit& hit : r.points)
                        ef_point(e, fid, hit.t, Point(hit.u, hit.v, 0));
                    // A run that ends inside the face leaves the surface there; one that ends on or beyond its boundary is cut by EE and VE.
                    for (const curve_surface_intersection::Overlap& run : r.overlaps)
                        for (bool last : {false, true}) {
                            const double t = last ? run.t1 : run.t0;
                            bool joined = false;
                            for (const curve_surface_intersection::Overlap& other : r.overlaps)
                                joined = joined || (&other != &run && std::min(std::abs(other.t0 - t), std::abs(other.t1 - t)) <= 1e-9 * (edge.t1 - edge.t0));
                            if (!joined && f.region.state(last ? run.pcurve.point_at_end() : run.pcurve.point_at_start()) == 1)
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
        const NurbsCurve& c = ds.curve_of(edge);
        const NurbsSurface& s = ds.surface_of(f);
        const double tolerance = edge.tolerance + f.tolerance + ds.fuzzy, step = probe(c, edge, tolerance);
        if (t - step > edge.t0 && t + step < edge.t1) {
            const Point before = c.point_at(t - step), after = c.point_at(t + step);
            if (s.closest_point(before).distance(before) <= tolerance && s.closest_point(after).distance(after) <= tolerance)
                return;
        }
        const int v = vertex_at(e, f, c.point_at(t), edge.tolerance);
        add_pave(e, v, t);
        // The common part of a crossing runs where the edge stays within the tolerance of the surface.
        Vector tangent = c.tangent_at(t), normal = s.normal_at(uv[0], uv[1]);
        tangent.normalize_self();
        normal.normalize_self();
        Vertex& made = ds.vertices[ds.real(v)];
        if (made.operand < 0)
            made.reach = std::max(made.reach, std::min(tolerance / std::max(std::abs(tangent.dot(normal)), 1e-12), c.point_at(edge.t0).distance(c.point_at(edge.t1))));
        if (state == 1 && std::find(f.in_vertices.begin(), f.in_vertices.end(), v) == f.in_vertices.end())
            f.in_vertices.push_back(v);
    }

    /// IntTools_EdgeFace within the tolerance: runs of an edge lying on the face and isolated touches the exact intersection does not see
    void tolerant_ef(int e, int fid, double tolerance, const curve_surface_intersection::Result& exact) {
        const Edge& edge = ds.edges[e];
        const Face& f = ds.faces[fid];
        const NurbsCurve& c = ds.curve_of(edge);
        const NurbsSurface& s = ds.surface_of(f);
        const int n = std::clamp(4 * c.cv_count() + 8, 16, 96);
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
                    for (const auto& [end, inner] : {std::pair{a, i > 0}, std::pair{b, j < n}}) {
                        double distance;
                        Point uv;
                        measure(end, distance, uv);
                        if (inner && f.region.state(uv) == 1)
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
        const NurbsCurve& c = ds.block_curve(b);
        const double lo = std::min(b.first.t, b.last.t), hi = std::max(b.first.t, b.last.t);
        double t = c.closest_parameter(p);
        if (t < lo || t > hi) {
            // A closest point outside the range may still have a near twin inside it on a closed curve.
            NurbsCurve piece = c;
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
                const NurbsSurface& s = ds.surface_of(f);
                std::vector<int> known = face_vertices(f);
                std::sort(known.begin(), known.end());
                const bool flat = s.degree(0) == 1 && s.degree(1) == 1;
                for (int e : act_e[o]) {
                    const Edge& edge = ds.edges[e];
                    if (edge.degenerated || !edge.box.overlaps(f.box))
                        continue;
                    const NurbsCurve& c = ds.curve_of(edge);
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
                        const int n = flat && c.degree() == 1 ? 2 : 8;
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
                        ds.pcurves[{b, fid}] = project(c, block.first.t, block.last.t, s);
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
                    if (chain >= 0 && !origins.count(chain) && loop_origin(ia, ib, r.curves, loop, chain, origin))
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
        int missed = 0;
        for (int v : both)
            missed += ds.vertices[v].reach > 0.0 && std::find(used.begin(), used.end(), v) == used.end();
        if (missed > 0)
            ds.log.push_back("FF incomplete: " + std::to_string(missed) + " edge crossings on no section");
    }

    /// The edges of a face lying inside a face on the same surface are In edges of it, whatever EF found
    void same_domain_in_blocks(int target, int source) {
        Face& f = ds.faces[target];
        const NurbsSurface& s = ds.surface_of(f);
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
                ds.pcurves[{b, target}] = project(ds.curve_of(edge), block.first.t, block.last.t, s);
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
                ends.push_back(last ? c.curve.point_at_end() : c.curve.point_at_start());
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
        const Point fallback = curves[chain].curve.point_at_start();
        for (const auto& [flat, other] : {std::pair{ia, ib}, std::pair{ib, ia}}) {
            Point o;
            Vector a, b;
            if (!affine_patch(ds.surface_of(ds.faces[flat]), o, a, b))
                continue;
            const NurbsSurface& s = ds.surface_of(ds.faces[other]);
            nurbs_implicit::Implicit form;
            try {
                form = nurbs_implicit::recognize(s, std::max(ds.faces[other].tolerance, confusion));
            } catch (const std::exception&) {
                continue;
            }
            Vector n = a.cross(b), axis(form.axis[0], form.axis[1], form.axis[2]), x;
            n.normalize_self();
            const Point c(form.point[0], form.point[1], form.point[2]);
            Point center;
            double radius = 0.0;
            if (form.kind == 2) {
                const double d = Vector(c[0] - o[0], c[1] - o[1], c[2] - o[2]).dot(n);
                center = Point(c[0] - n[0] * d, c[1] - n[1] * d, c[2] - n[2] * d);
                radius = std::sqrt(std::max(form.radius * form.radius - d * d, 0.0));
                x = canonical(n);
            } else if (form.kind == 3 || form.kind == 4) {
                const int d = s.degree(0) == 1 ? 0 : 1;
                const double u = 0.5 * (s.domain(1 - d).first + s.domain(1 - d).second);
                const Point p0 = d == 0 ? s.point_at(s.domain(0).first, u) : s.point_at(u, s.domain(1).first), p1 = d == 0 ? s.point_at(s.domain(0).second, u) : s.point_at(u, s.domain(1).second);
                if (axis.dot(Vector(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])) < 0)
                    axis *= -1.0;
                const double cosine = axis.dot(n);
                if (std::abs(cosine) < 1e-9)
                    continue;
                const double k = Vector(o[0] - c[0], o[1] - c[1], o[2] - c[2]).dot(n) / cosine;
                center = Point(c[0] + axis[0] * k, c[1] + axis[1] * k, c[2] + axis[2] * k);
                if (axis.cross(n).magnitude() < 1e-9) {
                    radius = center.distance(fallback);
                    x = canonical(axis);
                } else if (form.kind == 3) {
                    radius = form.radius / std::abs(cosine);
                    x = axis - n * cosine;
                    x.normalize_self();
                } else
                    continue;
            } else
                continue;
            origin = Point(center[0] + x[0] * radius, center[1] + x[1] * radius, center[2] + x[2] * radius);
            for (size_t i = 0; i < curves.size(); ++i)
                if (loop[i] == chain && curves[i].curve.closest_point(origin).distance(origin) <= std::max(curves[i].tolerance, confusion) + ds.fuzzy)
                    return true;
        }
        return false;
    }

    /// The X direction gp_Ax2 gets from a main direction alone (DirToAx2)
    static Vector canonical(const Vector& d) {
        const double x = std::abs(d[0]) < 1e-12 ? 0.0 : d[0], y = std::abs(d[1]) < 1e-12 ? 0.0 : d[1], z = std::abs(d[2]) < 1e-12 ? 0.0 : d[2];
        const double ax = std::abs(x), ay = std::abs(y), az = std::abs(z);
        Vector r = ax == 0.0 || (ax < ay && ax < az) ? Vector(0, -z, y) : ay == 0.0 || (ay < ax && ay < az) ? Vector(-z, 0, x) : Vector(-y, x, 0);
        r.normalize_self();
        return r;
    }

    void make_section(int ia, int ib, bop_ff::Curve& section, const Point* origin) {
        Curve c{ia, ib, std::move(section.curve), std::move(section.pcurve_a), std::move(section.pcurve_b), std::max(section.tolerance, confusion), {}};
        const NurbsCurve& w = c.world;
        const double t0 = w.domain_start(), t1 = w.domain_end();
        const bool closed = w.point_at_start().distance(w.point_at_end()) <= c.tolerance;
        std::vector<int> candidates;
        for (int f : {ia, ib}) {
            std::vector<int> more = face_vertices(ds.faces[f]);
            candidates.insert(candidates.end(), more.begin(), more.end());
            for (int b : face_blocks(ds.faces[f]))
                candidates.insert(candidates.end(), {ds.real(ds.blocks[b].first.vertex), ds.real(ds.blocks[b].last.vertex)});
        }
        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
        Box box = curve_box(w, c.tolerance);
        for (int v : candidates) {
            Box ball;
            ball.add(at(v), std::max(tol(v), ds.vertices[ds.real(v)].reach) + ds.fuzzy);
            if (!ball.overlaps(box))
                continue;
            const double t = w.closest_parameter(at(v));
            const double d = w.point_at(t).distance(at(v));
            if (d > std::max(tol(v), ds.vertices[ds.real(v)].reach) + c.tolerance + ds.fuzzy)
                continue;
            grow(v, d);
            c.paves.push_back({v, t});
            if (closed && (std::abs(t - t0) < 1e-9 * (t1 - t0) || std::abs(t - t1) < 1e-9 * (t1 - t0)))
                c.paves.push_back({v, std::abs(t - t0) < std::abs(t - t1) ? t1 : t0});
        }
        const auto near = [&](double t) {
            for (const Pave& p : c.paves)
                if (at(p.vertex).distance(w.point_at(t)) <= tol(p.vertex) + c.tolerance)
                    return true;
            return false;
        };
        if (!near(t0)) {
            const int v = ds.add_vertex(w.point_at(t0), c.tolerance);
            ds.vertices[v].joint = true;
            c.paves.push_back({v, t0});
            if (closed)
                c.paves.push_back({v, t1});
        }
        if (!near(t1)) {
            c.paves.push_back({ds.add_vertex(w.point_at(t1), c.tolerance), t1});
            ds.vertices.back().joint = true;
        }
        // PutBoundPaveOnCurve: a closed curve keeps a vertex at its bound unless one sits there already.
        const double bound = origin ? w.closest_parameter(*origin) : t0;
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
            const Curve& cc = ds.curves[curve];
            const NurbsCurve pa = trimmed(cc.pcurve_a, {t0, t1}, block.first.t, block.last.t);
            const NurbsCurve pb = trimmed(cc.pcurve_b, {t0, t1}, block.first.t, block.last.t);
            if (ds.faces[ia].region.state(pa.point_at(pa.domain_middle())) < 0 || ds.faces[ib].region.state(pb.point_at(pb.domain_middle())) < 0)
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
                    else if (key(old) == key(block) && (w.degree() > 1 || ds.block_curve(old).degree() > 1)) {
                        const NurbsCurve& oc = ds.block_curve(old);
                        Vector t1 = w.tangent_at(0.5 * (block.first.t + block.last.t)), t2 = oc.tangent_at(oc.closest_parameter(mid));
                        t1.normalize_self();
                        t2.normalize_self();
                        if (std::abs(t1.dot(t2)) >= 0.9063)
                            reach = std::max(reach, 2.0 * std::min({1e-3, 10.0 * (c.tolerance + ds.fuzzy), std::max(c.tolerance + ds.fuzzy, fat)}));
                    }
                    bool on = existing < 0;
                    for (int v : {block.first.vertex, block.last.vertex})
                        on = on && (v == old.first.vertex || v == old.last.vertex || on_block(old, at(v), reach + tol(v)));
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
                    const NurbsCurve& p = f == ia ? cc.pcurve_a : cc.pcurve_b;
                    ds.pcurves[{existing, f}] = !same ? project(ds.block_curve(kept), kept.first.t, kept.last.t, ds.surface_of(ds.faces[f]))
                        : along ? trimmed(p, {t0, t1}, block.first.t, block.last.t) : trimmed(p, {t0, t1}, block.last.t, block.first.t);
                }
        }
    }
};

}

DS fill(const BRep& a, const BRep& b, double fuzzy) {
    DS ds;
    ds.fuzzy = fuzzy;
    Filler filler{ds, {}, {}, {}};
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
