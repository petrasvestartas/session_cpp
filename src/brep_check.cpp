#include "brep_check.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>

namespace session_cpp::brep_check {

namespace {

using UV = std::array<double, 2>;

std::string num(double x) {
    char text[32];
    std::snprintf(text, sizeof text, "%.3g", x);
    return text;
}

bool finite(const Point& p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

bool good(double tolerance) {
    return std::isfinite(tolerance) && tolerance >= 0.0;
}

bool in(int index, size_t count) {
    return index >= 0 && index < (int)count;
}

bool bad_range(const std::array<double, 2>& range) {
    return range[0] > range[1] || (range[0] == range[1] && range[0] != 0.0);
}

double cross(const UV& o, const UV& a, const UV& b) {
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

// A pcurve use of an edge in a wire of a face, sampled in the direction the wire runs.
struct Use {
    int edge = -1;
    int wire = -1;
    int first = -1;
    int last = -1;
    bool seam = false;
    std::vector<UV> uv;
    int curve = -1;
    double s0 = 0.0, s1 = 0.0; // The pcurve parameter at the first and the last sample.
};

// Above a tolerance by more than rounding: tolerances are often set to the very gap measured.
const double slack = 1.001;

struct Checker {
    const BRep& b;
    const Options& o;
    Report report;
    std::vector<double> edge_size;
    std::vector<double> integral;

    void add(Status status, Entity entity, int index, double value, const std::string& message) {
        report.issues.push_back({status, entity, index, value, message});
    }

    // A tolerance with the precision of the geometry added, as BRepLib_ValidateEdge does.
    double tol(double tolerance) const {
        return tolerance + o.min_tolerance;
    }

    UV uv(int curve, double s) const {
        const Point p = b.m_curves_2d[curve].point_at(s);
        return {p[0], p[1]};
    }

    Point lifted(int surface, int curve, double s) const {
        const UV p = uv(curve, s);
        return b.m_surfaces[surface].point_at(p[0], p[1]);
    }

    bool references() {
        const size_t issues = report.issues.size();
        for (size_t i = 0; i < b.m_surfaces.size(); ++i)
            if (!b.m_surfaces[i].is_valid())
                add(Status::BadReference, Entity::Face, -1, (double)i, "surface " + std::to_string(i) + " is invalid");
        for (size_t i = 0; i < b.m_curves_3d.size(); ++i)
            if (!b.m_curves_3d[i].is_valid())
                add(Status::BadReference, Entity::Edge, -1, (double)i, "curve " + std::to_string(i) + " is invalid");
        for (size_t i = 0; i < b.m_curves_2d.size(); ++i)
            if (!b.m_curves_2d[i].is_valid())
                add(Status::BadReference, Entity::Edge, -1, (double)i, "pcurve " + std::to_string(i) + " is invalid");
        for (size_t i = 0; i < b.m_vertices.size(); ++i)
            if (!finite(b.m_vertices[i].point) || !good(b.m_vertices[i].tolerance))
                add(Status::BadReference, Entity::Vertex, (int)i, 0.0, "vertex " + std::to_string(i) + " has a non-finite point or a bad tolerance");
        for (size_t i = 0; i < b.m_edges.size(); ++i) {
            const BRepEdge& e = b.m_edges[i];
            bool ok = good(e.tolerance) && in(e.start_vertex, b.m_vertices.size()) && in(e.end_vertex, b.m_vertices.size());
            ok = ok && (e.degenerated ? e.start_vertex == e.end_vertex : in(e.curve_3d_index, b.m_curves_3d.size()));
            for (const BRepCurveOnSurface& pc : e.pcurves)
                ok = ok && in(pc.surface_index, b.m_surfaces.size()) && in(pc.curve_2d_index, b.m_curves_2d.size())
                    && (pc.curve_2d_index_2 < 0 || in(pc.curve_2d_index_2, b.m_curves_2d.size()));
            if (!ok)
                add(Status::BadReference, Entity::Edge, (int)i, 0.0, "edge " + std::to_string(i) + " has a bad reference or tolerance");
        }
        for (size_t i = 0; i < b.m_wires.size(); ++i)
            if (b.m_wires[i].edges.empty() || std::any_of(b.m_wires[i].edges.begin(), b.m_wires[i].edges.end(), [&](const BRepRef& r) { return !in(r.index, b.m_edges.size()); }))
                add(Status::BadReference, Entity::Wire, (int)i, 0.0, "wire " + std::to_string(i) + " is empty or has a bad edge");
        for (size_t i = 0; i < b.m_faces.size(); ++i) {
            const BRepFace& f = b.m_faces[i];
            if (!good(f.tolerance) || !in(f.surface_index, b.m_surfaces.size()) || f.wires.empty()
                || std::any_of(f.wires.begin(), f.wires.end(), [&](const BRepRef& r) { return !in(r.index, b.m_wires.size()); }))
                add(Status::BadReference, Entity::Face, (int)i, 0.0, "face " + std::to_string(i) + " has a bad surface, wire or tolerance");
        }
        for (size_t i = 0; i < b.m_shells.size(); ++i)
            if (b.m_shells[i].faces.empty() || std::any_of(b.m_shells[i].faces.begin(), b.m_shells[i].faces.end(), [&](const BRepRef& r) { return !in(r.index, b.m_faces.size()); }))
                add(Status::BadReference, Entity::Shell, (int)i, 0.0, "shell " + std::to_string(i) + " is empty or has a bad face");
        for (size_t i = 0; i < b.m_solids.size(); ++i)
            if (b.m_solids[i].shells.empty() || std::any_of(b.m_solids[i].shells.begin(), b.m_solids[i].shells.end(), [&](const BRepRef& r) { return !in(r.index, b.m_shells.size()); }))
                add(Status::BadReference, Entity::Solid, (int)i, 0.0, "solid " + std::to_string(i) + " is empty or has a bad shell");
        return report.issues.size() == issues;
    }

    void vertex_off(int vertex, int edge, const Point& end, const std::string& what) {
        const BRepVertex& v = b.m_vertices[vertex];
        const double d = v.point.distance(end, 0.0);
        if (d > slack * tol(v.tolerance))
            add(Status::VertexOffEdge, Entity::Vertex, vertex, d, "vertex " + std::to_string(vertex) + " off " + what + " of edge " + std::to_string(edge) + " by " + num(d) + " > " + num(tol(v.tolerance)));
    }

    void edges() {
        const int n = o.samples;
        edge_size.assign(b.m_edges.size(), 0.0);
        for (int i = 0; i < (int)b.m_edges.size(); ++i) {
            const BRepEdge& e = b.m_edges[i];
            if (bad_range(e.range) && !e.degenerated)
                add(Status::BadRange, Entity::Edge, i, e.range[1] - e.range[0], "edge " + std::to_string(i) + " range [" + num(e.range[0]) + ", " + num(e.range[1]) + "] is empty or reversed");
            if (e.degenerated && e.pcurves.empty())
                add(Status::DegeneratedWithoutPCurve, Entity::Edge, i, 0.0, "degenerated edge " + std::to_string(i) + " has no pcurve");
            const auto [t0, t1] = b.edge_range(i);
            if (!e.degenerated && !(t1 > t0)) {
                if (!bad_range(e.range))
                    add(Status::BadRange, Entity::Edge, i, t1 - t0, "edge " + std::to_string(i) + " runs over an empty range");
                continue;
            }
            std::vector<Point> points;
            if (!e.degenerated) {
                const Curve& c = b.m_curves_3d[e.curve_3d_index];
                for (int k = 0; k <= n; ++k)
                    points.push_back(c.point_at(t0 + (t1 - t0) * k / n));
                for (int k = 1; k <= n; ++k)
                    edge_size[i] += points[k].distance(points[k - 1], 0.0);
                vertex_off(e.start_vertex, i, points.front(), "the start");
                vertex_off(e.end_vertex, i, points.back(), "the end");
            }
            for (const BRepCurveOnSurface& pc : e.pcurves)
                for (int second = 0; second < (pc.curve_2d_index_2 >= 0 ? 2 : 1); ++second) {
                    const int curve = second ? pc.curve_2d_index_2 : pc.curve_2d_index;
                    if (bad_range(second ? pc.range_2 : pc.range)) {
                        add(Status::BadRange, Entity::Edge, i, 0.0, "pcurve " + std::to_string(curve) + " of edge " + std::to_string(i) + " has an empty or reversed range");
                        continue;
                    }
                    const auto [s0, s1] = b.pcurve_range(pc, second);
                    if (!(s1 > s0)) {
                        add(Status::BadRange, Entity::Edge, i, s1 - s0, "pcurve " + std::to_string(curve) + " of edge " + std::to_string(i) + " runs over an empty range");
                        continue;
                    }
                    double worst = 0.0, apart = 0.0;
                    std::vector<Point> lift;
                    for (int k = 0; k <= n; ++k) {
                        lift.push_back(lifted(pc.surface_index, curve, s0 + (s1 - s0) * k / n));
                        worst = std::max(worst, e.degenerated ? b.m_vertices[e.start_vertex].point.distance(lift[k], 0.0) : points[k].distance(lift[k], 0.0));
                    }
                    const double limit = tol(e.degenerated ? std::max(e.tolerance, b.m_vertices[e.start_vertex].tolerance) : e.tolerance);
                    if (worst > slack * limit && !e.degenerated)
                        for (const Point& q : lift)
                            apart = std::max(apart, b.m_curves_3d[e.curve_3d_index].closest_point(q).distance(q, 0.0));
                    if (worst > slack * limit && apart <= slack * limit && !e.degenerated) {
                        add(Status::SameParameter, Entity::Edge, i, worst, "edge " + std::to_string(i) + " lies on its pcurve " + std::to_string(curve) + " but apart from it in parameter by " + num(worst) + " > " + num(limit));
                        continue;
                    }
                    if (!e.degenerated) {
                        vertex_off(e.start_vertex, i, lift.front(), "the start of pcurve " + std::to_string(curve));
                        vertex_off(e.end_vertex, i, lift.back(), "the end of pcurve " + std::to_string(curve));
                    }
                    if (worst > slack * limit)
                        add(Status::EdgeOffPCurve, Entity::Edge, i, worst, "edge " + std::to_string(i) + " off its pcurve " + std::to_string(curve) + " on surface " + std::to_string(pc.surface_index) + " by " + num(worst) + " > " + num(limit));
                }
        }
    }

    void wires() {
        for (int w = 0; w < (int)b.m_wires.size(); ++w) {
            const std::vector<BRepRef> uses = b.wire_edges({w, BRepOrientation::Forward});
            int start = -1, previous = -1, previous_edge = -1;
            for (const BRepRef& r : uses) {
                if (r.orientation == BRepOrientation::Internal || r.orientation == BRepOrientation::External)
                    continue;
                const BRepEdge& e = b.m_edges[r.index];
                const bool reversed = r.orientation == BRepOrientation::Reversed;
                const int first = reversed ? e.end_vertex : e.start_vertex;
                if (previous >= 0 && previous != first)
                    add(Status::WireOpen, Entity::Wire, w, 0.0, "edges " + std::to_string(previous_edge) + " and " + std::to_string(r.index) + " of wire " + std::to_string(w) + " share no vertex");
                if (start < 0)
                    start = first;
                previous = reversed ? e.start_vertex : e.end_vertex;
                previous_edge = r.index;
            }
            if (start >= 0 && previous != start)
                add(Status::WireOpen, Entity::Wire, w, 0.0, "wire " + std::to_string(w) + " does not close");
        }
    }

    // The face's uses in wire order; false when an edge has no pcurve on its surface.
    bool sample(int f, std::vector<Use>& uses, std::vector<int>& wire_start, std::vector<bool>& broken) {
        const BRepFace& face = b.m_faces[f];
        bool complete = true;
        broken.assign(face.wires.size(), false);
        for (int w = 0; w < (int)face.wires.size(); ++w) {
            wire_start.push_back((int)uses.size());
            for (const BRepRef& r : b.wire_edges(face.wires[w])) {
                if (r.orientation == BRepOrientation::Internal || r.orientation == BRepOrientation::External)
                    continue;
                const BRepEdge& e = b.m_edges[r.index];
                const auto pc = std::find_if(e.pcurves.begin(), e.pcurves.end(), [&](const BRepCurveOnSurface& p) { return p.surface_index == face.surface_index; });
                if (pc == e.pcurves.end()) {
                    if (!e.pcurves.empty() || !e.degenerated)
                        add(Status::EdgeWithoutPCurve, Entity::Edge, r.index, 0.0, "edge " + std::to_string(r.index) + " has no pcurve on the surface of face " + std::to_string(f));
                    complete = false;
                    broken[w] = true;
                    continue;
                }
                const bool reversed = r.orientation == BRepOrientation::Reversed;
                const bool second = reversed && pc->curve_2d_index_2 >= 0;
                const int curve = second ? pc->curve_2d_index_2 : pc->curve_2d_index;
                const auto [s0, s1] = b.pcurve_range(*pc, second);
                if (!(s1 > s0) || bad_range(second ? pc->range_2 : pc->range)) {
                    complete = false;
                    broken[w] = true;
                    continue;
                }
                Use use{r.index, w, reversed ? e.end_vertex : e.start_vertex, reversed ? e.start_vertex : e.end_vertex, pc->curve_2d_index_2 >= 0, {}, curve, reversed ? s1 : s0, reversed ? s0 : s1};
                for (int k = 0; k <= o.samples; ++k) {
                    const double s = s0 + (s1 - s0) * k / o.samples;
                    use.uv.push_back(uv(curve, reversed ? s0 + s1 - s : s));
                }
                uses.push_back(std::move(use));
            }
        }
        wire_start.push_back((int)uses.size());
        return complete;
    }

    double integrand(const Surface& s, const Vector& centre, double u, double v) const {
        const std::vector<Vector> d = s.evaluate(u, v, 1);
        return (d[0] - centre).dot(d[2].cross(d[1])) / 3.0;
    }

    // Green's theorem on the face's share of the volume: along u (dir 0) the loops integrate G dv with dG/du = g, along v they integrate -H du with dH/dv = g.
    double face_integral(const Surface& s, const std::vector<Use>& uses, const Vector& centre, const std::array<double, 4>& box, int dir) const {
        static const double x[8] = {-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498, 0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363};
        static const double w[8] = {0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620, 0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763};
        const double origin = box[2 * dir], span = std::max(box[2 * dir + 1] - box[2 * dir], 1e-300);
        auto primitive = [&](const UV& p) {
            const int chunks = std::max(1, (int)std::ceil(4.0 * std::abs(p[dir] - origin) / span));
            const double h = (p[dir] - origin) / chunks;
            double sum = 0.0;
            for (int c = 0; c < chunks; ++c)
                for (int i = 0; i < 8; ++i) {
                    const double t = origin + h * (c + 0.5 + 0.5 * x[i]);
                    sum += 0.5 * h * w[i] * (dir == 0 ? integrand(s, centre, t, p[1]) : integrand(s, centre, p[0], t));
                }
            return sum;
        };
        double total = 0.0;
        for (const Use& use : uses) {
            double before = primitive(use.uv[0]);
            for (size_t k = 1; k < use.uv.size(); ++k) {
                const double after = primitive(use.uv[k]);
                const double step = use.uv[k][1 - dir] - use.uv[k - 1][1 - dir];
                total += (dir == 0 ? 0.5 : -0.5) * (before + after) * step;
                before = after;
            }
        }
        return total;
    }

    // Whether the arcs of two pcurves under two crossing chords cross too, sampled sixteen times finer.
    bool refined(const std::vector<Use>& uses, int i, int k, int j, int l) const {
        const int m = 16;
        auto arc = [&](const Use& u, int seg) {
            std::vector<UV> out;
            for (int q = 0; q <= m; ++q)
                out.push_back(uv(u.curve, u.s0 + (u.s1 - u.s0) * (seg - 1 + (double)q / m) / o.samples));
            return out;
        };
        const std::vector<UV> p = arc(uses[i], k), q = arc(uses[j], l);
        for (int x = 1; x <= m; ++x)
            for (int y = 1; y <= m; ++y) {
                const double d1 = cross(p[x - 1], p[x], q[y - 1]), d2 = cross(p[x - 1], p[x], q[y]), d3 = cross(q[y - 1], q[y], p[x - 1]), d4 = cross(q[y - 1], q[y], p[x]);
                if (d1 * d2 <= 0.0 && d3 * d4 <= 0.0 && d1 != d2 && d3 != d4)
                    return true;
            }
        return false;
    }

    // Pcurves of a face crossing in uv away from the tolerance of their vertices and edges, doubled for the chords (BRepCheck_Wire::SelfIntersect).
    void crossings(int f, const Surface& surface, const std::vector<Use>& uses, const std::array<double, 4>& box) {
        struct Segment {
            UV a, b;
            int use, k;
        };
        std::vector<Segment> segments;
        for (int i = 0; i < (int)uses.size(); ++i)
            for (int k = 1; k < (int)uses[i].uv.size(); ++k)
                segments.push_back({uses[i].uv[k - 1], uses[i].uv[k], i, k});
        std::sort(segments.begin(), segments.end(), [](const Segment& p, const Segment& q) { return std::min(p.a[0], p.b[0]) < std::min(q.a[0], q.b[0]); });
        const double eps = 1e-4 * std::hypot(box[1] - box[0], box[3] - box[2]);
        std::set<Status> found;
        std::vector<int> active;
        for (int i = 0; i < (int)segments.size(); ++i) {
            const Segment& s = segments[i];
            const double lo = std::min(s.a[0], s.b[0]);
            std::erase_if(active, [&](int j) { return std::max(segments[j].a[0], segments[j].b[0]) < lo; });
            for (int j : active) {
                const Segment& t = segments[j];
                if (std::max(s.a[1], s.b[1]) < std::min(t.a[1], t.b[1]) || std::max(t.a[1], t.b[1]) < std::min(s.a[1], s.b[1]))
                    continue;
                if (s.use == t.use && std::abs(s.k - t.k) <= 1)
                    continue;
                const double d1 = cross(s.a, s.b, t.a), d2 = cross(s.a, s.b, t.b), d3 = cross(t.a, t.b, s.a), d4 = cross(t.a, t.b, s.b);
                if ((d1 > 0 && d2 > 0) || (d1 < 0 && d2 < 0) || (d3 > 0 && d4 > 0) || (d3 < 0 && d4 < 0) || d3 == d4 || d1 == d2)
                    continue;
                const double r = d3 / (d3 - d4);
                const UV p = {s.a[0] + r * (s.b[0] - s.a[0]), s.a[1] + r * (s.b[1] - s.a[1])};
                auto end = [&](const Segment& g) {
                    const int last = (int)uses[g.use].uv.size() - 1;
                    return (g.k == 1 && std::hypot(p[0] - g.a[0], p[1] - g.a[1]) < eps) || (g.k == last && std::hypot(p[0] - g.b[0], p[1] - g.b[1]) < eps);
                };
                const Use &a = uses[s.use], &c = uses[t.use];
                const Point x = surface.point_at(p[0], p[1]);
                bool inside = false;
                for (int v : {a.first, a.last, c.first, c.last})
                    inside = inside || x.distance(b.m_vertices[v].point, 0.0) <= 2.0 * tol(std::max({b.m_vertices[v].tolerance, b.m_edges[a.edge].tolerance, b.m_edges[c.edge].tolerance}));
                if (inside || end(s) || end(t) || !refined(uses, s.use, s.k, t.use, t.k))
                    continue;
                const Status status = s.use == t.use ? Status::PCurveSelfIntersect : a.wire == c.wire ? Status::LoopSelfIntersect : Status::LoopsIntersect;
                if (!found.insert(status).second)
                    continue;
                const int wire = b.m_faces[f].wires[a.wire].index;
                if (status == Status::PCurveSelfIntersect)
                    add(status, Entity::Edge, a.edge, 0.0, "pcurve of edge " + std::to_string(a.edge) + " crosses itself on face " + std::to_string(f));
                else if (status == Status::LoopSelfIntersect)
                    add(status, Entity::Wire, wire, 0.0, "edges " + std::to_string(a.edge) + " and " + std::to_string(c.edge) + " of wire " + std::to_string(wire) + " cross in uv on face " + std::to_string(f));
                else
                    add(status, Entity::Face, f, 0.0, "wires " + std::to_string(wire) + " and " + std::to_string(b.m_faces[f].wires[c.wire].index) + " of face " + std::to_string(f) + " cross in uv");
            }
            active.push_back(i);
        }
    }

    void faces() {
        Point lo(1e300, 1e300, 1e300), hi(-1e300, -1e300, -1e300);
        for (const BRepVertex& v : b.m_vertices)
            for (int a = 0; a < 3; ++a) {
                lo[a] = std::min(lo[a], v.point[a]);
                hi[a] = std::max(hi[a], v.point[a]);
            }
        const Vector centre = b.m_vertices.empty() ? Vector(0, 0, 0) : Vector(0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2]));
        integral.assign(b.m_faces.size(), std::nan(""));
        for (int f = 0; f < (int)b.m_faces.size(); ++f) {
            const Surface& s = b.m_surfaces[b.m_faces[f].surface_index];
            std::vector<Use> uses;
            std::vector<int> wire_start;
            std::vector<bool> broken;
            bool complete = sample(f, uses, wire_start, broken);
            if (uses.empty())
                continue;
            std::array<double, 4> box = {1e300, -1e300, 1e300, -1e300};
            for (const Use& use : uses)
                for (const UV& p : use.uv)
                    for (int d = 0; d < 2; ++d) {
                        box[2 * d] = std::min(box[2 * d], p[d]);
                        box[2 * d + 1] = std::max(box[2 * d + 1], p[d]);
                    }
            std::array<double, 2> period = {s.period(0), s.period(1)};
            for (int a = 0; a < 2; ++a)
                if (period[a] == 0.0 && s.kind() == Surface::Kind::Nurbs && s.nurbs().is_closed(a))
                    period[a] = s.domain(a).second - s.domain(a).first;
            const bool seams = std::any_of(uses.begin(), uses.end(), [](const Use& use) { return use.seam; });
            std::array<bool, 2> ring = {false, false};
            for (int w = 0; w + 1 < (int)wire_start.size(); ++w) {
                const int first = wire_start[w], end = wire_start[w + 1];
                if (first == end || broken[w])
                    continue;
                bool closed = complete;
                for (int i = first; i < end; ++i) {
                    const Use& p = uses[i];
                    const Use& q = uses[i + 1 < end ? i + 1 : first];
                    const UV &x = p.uv.back(), &y = q.uv.front();
                    const double du = std::abs(y[0] - x[0]), dv = std::abs(y[1] - x[1]);
                    if (du <= 0.01 * (box[1] - box[0]) && dv <= 0.01 * (box[3] - box[2]))
                        continue;
                    const std::vector<Vector> d = s.evaluate(0.5 * (x[0] + y[0]), 0.5 * (x[1] + y[1]), 1);
                    const double limit = tol(b.m_vertices[p.last].tolerance);
                    if (du * d[2].magnitude() <= limit && dv * d[1].magnitude() <= limit)
                        continue;
                    bool modulo = true;
                    for (int a = 0; a < 2; ++a) {
                        const double gap = a == 0 ? du : dv;
                        const double turns = period[a] > 0.0 ? std::round(gap / period[a]) : 0.0;
                        modulo = modulo && std::abs(gap - turns * period[a]) <= 0.01 * std::max(period[a], box[2 * a + 1] - box[2 * a]);
                        ring[a] = ring[a] || turns != 0.0;
                    }
                    closed = false;
                    if (modulo && !seams)
                        continue;
                    const int wire = b.m_faces[f].wires[p.wire].index;
                    add(Status::WireOpenUV, Entity::Wire, wire, std::max(du, dv), "wire " + std::to_string(wire) + " of face " + std::to_string(f) + (modulo ? " closes in uv only modulo a period" : " is open in uv") + " between edges " + std::to_string(p.edge) + " and " + std::to_string(q.edge) + " by (" + num(du) + ", " + num(dv) + ")");
                    if (!modulo)
                        complete = false;
                }
                if (!closed || w != 0)
                    continue;
                double area = 0.0;
                for (int i = first; i < end; ++i)
                    for (size_t k = 1; k < uses[i].uv.size(); ++k)
                        area += uses[i].uv[k - 1][0] * uses[i].uv[k][1] - uses[i].uv[k][0] * uses[i].uv[k - 1][1];
                for (int i = first; i < end; ++i) {
                    const UV &x = uses[i].uv.back(), &y = uses[i + 1 < end ? i + 1 : first].uv.front();
                    area += x[0] * y[1] - y[0] * x[1];
                }
                area *= 0.5;
                if (area < -1e-12 * (box[1] - box[0]) * (box[3] - box[2])) {
                    const int wire = b.m_faces[f].wires[w].index;
                    add(Status::LoopOrientation, Entity::Wire, wire, area, "outer wire " + std::to_string(wire) + " of face " + std::to_string(f) + " runs clockwise in uv");
                }
            }
            std::map<int, int> twice;
            for (const Use& use : uses)
                if (!use.seam && !b.m_edges[use.edge].degenerated && ++twice[use.edge] == 2)
                    add(Status::SeamBoundary, Entity::Face, f, use.edge, "edge " + std::to_string(use.edge) + " bounds face " + std::to_string(f) + " on both sides with one pcurve: a boundary on its own seam that is no seam pair");
            crossings(f, s, uses, box);
            if (complete && !(ring[0] && ring[1]))
                integral[f] = face_integral(s, uses, centre, box, ring[0] ? 1 : 0);
        }
    }

    void shells() {
        std::vector<bool> in_solid(b.m_shells.size(), false);
        for (const BRepSolid& solid : b.m_solids)
            for (const BRepRef& r : solid.shells)
                in_solid[r.index] = true;
        std::vector<bool> sound(b.m_shells.size(), true);
        for (int s = 0; s < (int)b.m_shells.size(); ++s) {
            std::vector<int> uses(b.m_edges.size(), 0), balance(b.m_edges.size(), 0);
            for (const BRepRef& face : b.m_shells[s].faces)
                for (const BRepRef& wire : b.m_faces[face.index].wires)
                    for (const BRepRef& edge : b.wire_edges(wire)) {
                        const BRepOrientation orientation = brep_compose(face.orientation, edge.orientation);
                        if (b.m_edges[edge.index].degenerated || orientation == BRepOrientation::Internal || orientation == BRepOrientation::External)
                            continue;
                        ++uses[edge.index];
                        balance[edge.index] += orientation == BRepOrientation::Forward ? 1 : -1;
                    }
            int free = 0, twisted = 0;
            for (int e = 0; e < (int)b.m_edges.size(); ++e) {
                if ((uses[e] > 2 && balance[e] != 0) || (in_solid[s] && uses[e] % 2 == 1))
                    add(Status::EdgeUses, Entity::Edge, e, uses[e], "edge " + std::to_string(e) + " used " + std::to_string(uses[e]) + " times in shell " + std::to_string(s));
                free += uses[e] % 2 == 1;
                twisted += uses[e] % 2 == 0 && balance[e] != 0;
            }
            if (in_solid[s] && free > 0)
                add(Status::ShellOpen, Entity::Shell, s, free, "shell " + std::to_string(s) + " has " + std::to_string(free) + " free edges");
            if (twisted > 0)
                add(Status::ShellNotOrientable, Entity::Shell, s, twisted, "shell " + std::to_string(s) + " uses " + std::to_string(twisted) + " edges twice in the same direction");
            sound[s] = free == 0 && twisted == 0 && std::all_of(uses.begin(), uses.end(), [](int n) { return n % 2 == 0; });
        }
        for (int i = 0; i < (int)b.m_solids.size(); ++i) {
            double volume = 0.0;
            bool known = true;
            for (const BRepRef& shell : b.m_solids[i].shells) {
                known = known && sound[shell.index];
                for (const BRepRef& face : b.m_shells[shell.index].faces) {
                    known = known && !std::isnan(integral[face.index]);
                    volume += (brep_compose(shell.orientation, face.orientation) == BRepOrientation::Reversed ? -1.0 : 1.0) * integral[face.index];
                }
            }
            if (known && volume < 0.0)
                add(Status::SolidInsideOut, Entity::Solid, i, volume, "solid " + std::to_string(i) + " encloses a negative volume " + num(volume));
        }
    }

    void proportions() {
        std::vector<double> all;
        std::vector<double> vertex_size(b.m_vertices.size(), INFINITY);
        for (int e = 0; e < (int)b.m_edges.size(); ++e) {
            if (b.m_edges[e].degenerated)
                continue;
            all.push_back(tol(b.m_edges[e].tolerance));
            for (int v : {b.m_edges[e].start_vertex, b.m_edges[e].end_vertex})
                vertex_size[v] = std::min(vertex_size[v], edge_size[e]);
        }
        for (const BRepVertex& v : b.m_vertices)
            all.push_back(tol(v.tolerance));
        if (all.empty())
            return;
        std::nth_element(all.begin(), all.begin() + all.size() / 2, all.end());
        const double median = all[all.size() / 2];
        auto test = [&](Entity entity, int index, double tolerance, double size, const std::string& what) {
            if (std::isfinite(size) && tolerance > o.size_fraction * size && tolerance > o.median_factor * median && tolerance > o.min_tolerance)
                add(Status::ToleranceOutOfProportion, entity, index, tolerance, what + " " + std::to_string(index) + " tolerance " + num(tolerance) + " against size " + num(size) + " and median " + num(median));
        };
        for (int e = 0; e < (int)b.m_edges.size(); ++e) {
            const BRepEdge& edge = b.m_edges[e];
            if (o.reference_tolerance > 0.0 && edge.tolerance > o.reference_factor * o.reference_tolerance)
                add(Status::ToleranceOutOfProportion, Entity::Edge, e, edge.tolerance, "edge " + std::to_string(e) + " tolerance " + num(edge.tolerance) + " above " + num(o.reference_factor) + " times the reference " + num(o.reference_tolerance));
            test(Entity::Edge, e, edge.tolerance, edge.degenerated ? 0.0 : edge_size[e], edge.degenerated ? "degenerated edge" : "edge");
        }
        for (int v = 0; v < (int)b.m_vertices.size(); ++v)
            test(Entity::Vertex, v, b.m_vertices[v].tolerance, vertex_size[v], "vertex");
    }
};

}

bool Report::has(Status status) const {
    return std::any_of(issues.begin(), issues.end(), [&](const Issue& i) { return i.status == status; });
}

std::string Report::summary(int first) const {
    if (issues.empty())
        return "valid";
    std::string out = std::to_string(issues.size()) + (issues.size() == 1 ? " issue: " : " issues: ");
    for (int i = 0; i < std::min(first, (int)issues.size()); ++i)
        out += (i ? "; " : "") + name(issues[i].status) + ", " + issues[i].message;
    return out;
}

std::string name(Status status) {
    static const char* names[] = {"BadReference", "VertexOffEdge", "EdgeOffPCurve", "SameParameter", "BadRange", "PCurveSelfIntersect", "LoopSelfIntersect", "LoopsIntersect", "WireOpen", "WireOpenUV",
        "LoopOrientation", "SeamBoundary", "ToleranceOutOfProportion", "EdgeUses", "ShellOpen", "ShellNotOrientable", "SolidInsideOut", "DegeneratedWithoutPCurve", "EdgeWithoutPCurve"};
    return names[(int)status];
}

double deviation(const BRep& b, int edge, int samples) {
    const BRepEdge& e = b.m_edges[edge];
    const auto [t0, t1] = b.edge_range(edge);
    if (e.degenerated || !(t1 > t0))
        return 0.0;
    double worst = 0.0;
    for (const BRepCurveOnSurface& pc : e.pcurves)
        for (int second = 0; second < (pc.curve_2d_index_2 >= 0 ? 2 : 1); ++second) {
            const auto [s0, s1] = b.pcurve_range(pc, second);
            if (!(s1 > s0) || bad_range(second ? pc.range_2 : pc.range))
                continue;
            for (int k = 0; k <= samples; ++k) {
                const Point p = b.m_curves_2d[second ? pc.curve_2d_index_2 : pc.curve_2d_index].point_at(s0 + (s1 - s0) * k / samples);
                worst = std::max(worst, b.m_curves_3d[e.curve_3d_index].point_at(t0 + (t1 - t0) * k / samples).distance(b.m_surfaces[pc.surface_index].point_at(p[0], p[1]), 0.0));
            }
        }
    return worst;
}

Report check(const BRep& brep, const Options& options) {
    Checker checker{brep, options, {}, {}, {}};
    if (!checker.references())
        return checker.report;
    checker.edges();
    checker.wires();
    checker.faces();
    checker.shells();
    checker.proportions();
    return checker.report;
}

}
