#include "bop_ds.h"
#include <algorithm>
#include <cmath>
#include <memory>

namespace session_cpp::bop {

void Box::add(const Point& p, double grow) {
    for (int d = 0; d < 3; ++d) {
        lo[d] = std::min(lo[d], p[d] - grow);
        hi[d] = std::max(hi[d], p[d] + grow);
    }
}

bool Box::overlaps(const Box& other, double gap) const {
    for (int d = 0; d < 3; ++d)
        if (lo[d] > other.hi[d] + gap || other.lo[d] > hi[d] + gap)
            return false;
    return true;
}

int Region::winding(const Point& from, const Point& to) const {
    for (int d = 0; d < 2; ++d)
        if (period[d] > 0.0 && std::abs(std::abs(to[d] - from[d]) - period[d]) < 0.25 * period[d])
            return d;
    return -1;
}

int Region::band() const {
    int axis = -1;
    for (int w : winds)
        axis = std::max(axis, w);
    return axis;
}

void Region::add_loop(std::vector<Point> loop, int winding) {
    winds.push_back(winding);
    for (const Point& p : loop)
        for (int d = 0; d < 2; ++d) {
            lo[d] = std::min(lo[d], p[d]);
            hi[d] = std::max(hi[d], p[d]);
        }
    loops.push_back(std::move(loop));
    arcs.emplace_back();
}

static double segment_distance(const Point& p, const Point& a, const Point& b) {
    const double dx = b[0] - a[0], dy = b[1] - a[1], len = dx * dx + dy * dy;
    double s = len > 0 ? ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len : 0.0;
    s = std::clamp(s, 0.0, 1.0);
    return std::hypot(p[0] - a[0] - s * dx, p[1] - a[1] - s * dy);
}

int Region::state(const Point& uv) const {
    if (const int axis = band(); axis >= 0) {
        // A band around a periodic surface: a ray across it from the point, against the loops and their copies one period either way.
        const int other = 1 - axis;
        Point q = uv;
        q[axis] -= std::floor((q[axis] - lo[axis]) / period[axis]) * period[axis];
        bool inside = false;
        for (size_t l = 0; l < loops.size(); ++l)
            for (size_t k = 0; k + (winds[l] >= 0 ? 1 : 0) < loops[l].size(); ++k)
                for (int copy = -1; copy <= 1; ++copy) {
                    Point a = loops[l][k], b = loops[l][(k + 1) % loops[l].size()];
                    a[axis] += copy * period[axis];
                    b[axis] += copy * period[axis];
                    if (segment_distance(q, a, b) <= eps)
                        return 0;
                    if ((a[axis] > q[axis]) != (b[axis] > q[axis]) && q[other] < a[other] + (q[axis] - a[axis]) / (b[axis] - a[axis]) * (b[other] - a[other]))
                        inside = !inside;
                }
        return inside ? 1 : -1;
    }
    int best = -1;
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j) {
            if ((i && !period[0]) || (j && !period[1]))
                continue;
            const Point q(uv[0] + i * period[0], uv[1] + j * period[1], 0);
            if (q[0] < lo[0] - eps || q[0] > hi[0] + eps || q[1] < lo[1] - eps || q[1] > hi[1] + eps)
                continue;
            bool inside = false, on = false;
            const auto visit = [&](const Point& a, const Point& b) {
                if (segment_distance(q, a, b) <= eps)
                    on = true;
                else if ((a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0] + (q[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]))
                    inside = !inside;
            };
            for (size_t l = 0; l < loops.size(); ++l) {
                const auto& loop = loops[l];
                for (size_t k = 0; k < loop.size() && !on; ++k) {
                    const Point& a = loop[k];
                    const Point& b = loop[(k + 1) % loop.size()];
                    // Near a segment that stands for a bulging pcurve, the pcurve itself decides.
                    if (arcs[l].size() != loop.size() || segment_distance(q, a, b) > 2.0 * arcs[l][k].bulge + eps) {
                        visit(a, b);
                        continue;
                    }
                    const Arc& arc = arcs[l][k];
                    const int n = 64;
                    Point from = a;
                    for (int i = 1; i <= n && !on; ++i) {
                        const Point to = i == n ? b : arc.curve->point_at(arc.t0 + (arc.t1 - arc.t0) * i / n);
                        visit(from, to);
                        from = to;
                    }
                }
            }
            best = std::max(best, on ? 0 : inside ? 1 : -1);
        }
    return best;
}

double Region::area() const {
    double sum = 0.0;
    for (const auto& loop : loops)
        for (size_t k = 0; k < loop.size(); ++k) {
            const Point& a = loop[k];
            const Point& b = loop[(k + 1) % loop.size()];
            sum += a[0] * b[1] - b[0] * a[1];
        }
    return 0.5 * sum;
}

bool Region::interior(Point& uv) const {
    // Scanlines of constant v; across a band, of constant position around it.
    const int axis = band(), held = axis >= 0 ? axis : 1, run = 1 - held;
    const double span = axis >= 0 ? period[axis] : hi[held] - lo[held];
    double widest = 0.0;
    for (double f : {0.5, 0.37, 0.63, 0.23, 0.77, 0.11, 0.89, 0.03, 0.97}) {
        const double v = lo[held] + f * span;
        std::vector<double> xs;
        for (size_t l = 0; l < loops.size(); ++l)
            for (size_t k = 0; k + (winds[l] >= 0 ? 1 : 0) < loops[l].size(); ++k)
                for (int copy = axis >= 0 ? -1 : 0; copy <= (axis >= 0 ? 1 : 0); ++copy) {
                    Point a = loops[l][k], b = loops[l][(k + 1) % loops[l].size()];
                    a[held] += axis >= 0 ? copy * period[axis] : 0.0;
                    b[held] += axis >= 0 ? copy * period[axis] : 0.0;
                    if ((a[held] > v) != (b[held] > v))
                        xs.push_back(a[run] + (v - a[held]) / (b[held] - a[held]) * (b[run] - a[run]));
                }
        std::sort(xs.begin(), xs.end());
        for (size_t k = 0; k + 1 < xs.size(); k += 2)
            if (xs[k + 1] - xs[k] > widest) {
                Point mid(0, 0, 0);
                mid[run] = 0.5 * (xs[k] + xs[k + 1]);
                mid[held] = v;
                if (state(mid) == 1) {
                    widest = xs[k + 1] - xs[k];
                    uv = mid;
                }
            }
        if (widest > 0.2 * (hi[run] - lo[run]))
            break;
    }
    return widest > 0.0;
}

static int samples(const NurbsCurve& c) {
    return std::clamp(4 * c.cv_count(), 8, 256);
}

std::array<double, 2> surface_period(const NurbsSurface& s) {
    std::array<double, 2> period{0.0, 0.0};
    for (int d = 0; d < 2; ++d) {
        const auto [lo, hi] = s.domain(d);
        const auto [a, b] = s.domain(1 - d);
        if (s.is_closed(d)) {
            period[d] = hi - lo;
            continue;
        }
        const auto at = [&](double x, double w) { return d == 0 ? s.point_at(x, w) : s.point_at(w, x); };
        const double size = at(lo, a).distance(at(hi, b)) + at(lo, b).distance(at(hi, a));
        for (int k = 0; k < s.nurbsknot_count(d) && period[d] == 0.0; ++k) {
            const double step = s.nurbsknot(d, k) - lo;
            if (step <= 1e-9 * (hi - lo) || step >= (1 - 1e-9) * (hi - lo))
                continue;
            bool same = true;
            for (double x : {lo, lo + 0.37 * (hi - lo - step), hi - step})
                for (double f : {0.13, 0.5, 0.91})
                    same = same && at(x, a + f * (b - a)).distance(at(x + step, a + f * (b - a))) <= 1e-9 * size;
            if (same)
                period[d] = step;
        }
    }
    return period;
}

Region face_region(const BRep& brep, int face, double tolerance) {
    Region region;
    const NurbsSurface& s = brep.m_surfaces[brep.m_faces[face].surface_index].nurbs();
    region.period = surface_period(brep.m_surfaces[brep.m_faces[face].surface_index]);
    for (const BRepRef& wire : brep.m_faces[face].wires) {
        std::vector<Point> loop;
        std::vector<Region::Arc> arcs;
        Point last;
        for (const BRepRef& use : brep.wire_edges(wire)) {
            const Span whole = pcurve_span(brep, use.index, face, use.orientation);
            if (!whole.curve)
                continue;
            const NurbsCurve& p = whole.curve->nurbs();
            const int n = samples(p);
            const double t0 = whole.t0, span = whole.t1 - t0;
            for (int i = 0; i < n; ++i) {
                const bool back = use.orientation == BRepOrientation::Reversed;
                const double f = back ? 1.0 - double(i) / n : double(i) / n, g = back ? 1.0 - double(i + 1) / n : double(i + 1) / n;
                const Point a = p.point_at(t0 + f * span), b = p.point_at(t0 + g * span);
                double bulge = 0.0;
                if (p.degree() > 1)
                    for (double k : {0.25, 0.5, 0.75})
                        bulge = std::max(bulge, segment_distance(p.point_at(t0 + (f + k * (g - f)) * span), a, b));
                loop.push_back(a);
                arcs.push_back({&p, t0 + f * span, t0 + g * span, bulge});
                last = b;
            }
        }
        // A wire without a seam edge around a periodic surface does not close in uv: it ends one period from where it began.
        const int winding = loop.empty() ? -1 : region.winding(loop.front(), last);
        if (winding >= 0) {
            loop.push_back(last);
            region.add_loop(std::move(loop), winding);
        } else if (loop.size() > 2) {
            region.add_loop(std::move(loop));
            region.arcs.back() = std::move(arcs);
        }
    }
    const double u = 0.5 * (s.domain(0).first + s.domain(0).second), v = 0.5 * (s.domain(1).first + s.domain(1).second);
    const std::vector<Vector> d = s.evaluate(u, v, 1);
    const double speed = std::max({d[1].magnitude(), d[2].magnitude(), 1e-300});
    region.eps = std::max(tolerance / speed, 1e-12 * std::max(region.hi[0] - region.lo[0], region.hi[1] - region.lo[1]));
    return region;
}

int DS::add_vertex(const Point& p, double tolerance) {
    vertices.push_back({p, tolerance, -1, -1, -1, 0.0, false});
    return static_cast<int>(vertices.size()) - 1;
}

int DS::merge(const std::vector<int>& group) {
    std::vector<int> real_ids;
    for (int v : group)
        if (std::find(real_ids.begin(), real_ids.end(), real(v)) == real_ids.end())
            real_ids.push_back(real(v));
    if (real_ids.size() == 1)
        return real_ids[0];
    Point c(0, 0, 0);
    for (int v : real_ids)
        for (int d = 0; d < 3; ++d)
            c[d] += vertices[v].point[d] / real_ids.size();
    double tolerance = 0.0;
    for (int v : real_ids)
        tolerance = std::max(tolerance, c.distance(vertices[v].point) + vertices[v].tolerance);
    const int merged = add_vertex(c, tolerance);
    vertices[merged].joint = true;
    for (int v : real_ids) {
        vertices[merged].operand = std::max(vertices[merged].operand, vertices[v].operand);
        vertices[merged].joint = vertices[merged].joint && vertices[v].joint;
        vertices[merged].reach = std::max(vertices[merged].reach, vertices[v].reach);
        vertices[v].sd = merged;
    }
    return merged;
}

Point DS::block_point(const PaveBlock& b, double s) const {
    if (b.edge >= 0 && edges[b.edge].degenerated)
        return vertices[real(b.first.vertex)].point;
    return block_curve(b).point_at(b.first.t + s * (b.last.t - b.first.t));
}

bool affine_patch(const NurbsSurface& surface, Point& origin, Vector& a, Vector& b) {
    if (surface.degree(0) != 1 || surface.degree(1) != 1 || surface.cv_count(0) != 2 || surface.cv_count(1) != 2 || surface.is_rational())
        return false;
    origin = surface.get_cv(0, 0);
    const Point pu = surface.get_cv(1, 0), pv = surface.get_cv(0, 1), q = surface.get_cv(1, 1);
    a = Vector(pu[0] - origin[0], pu[1] - origin[1], pu[2] - origin[2]);
    b = Vector(pv[0] - origin[0], pv[1] - origin[1], pv[2] - origin[2]);
    const double gap = Point(origin[0] + a[0] + b[0], origin[1] + a[1] + b[1], origin[2] + a[2] + b[2]).distance(q);
    return gap <= 1e-12 * std::sqrt(a.dot(a) + b.dot(b)) && a.cross(b).magnitude() > 0;
}

NurbsCurve project(const NurbsCurve& curve, double t0, double t1, const NurbsSurface& surface) {
    Point o;
    Vector a, b;
    if (affine_patch(surface, o, a, b)) {
        const double aa = a.dot(a), ab = a.dot(b), bb = b.dot(b), det = aa * bb - ab * ab;
        const auto [u0, u1] = surface.domain(0);
        const auto [v0, v1] = surface.domain(1);
        NurbsCurve uv = curve;
        if (std::min(t0, t1) > curve.domain_start() + 1e-12 * (curve.domain_end() - curve.domain_start()) || std::max(t0, t1) < curve.domain_end() - 1e-12 * (curve.domain_end() - curve.domain_start()))
            uv.trim(std::max(std::min(t0, t1), curve.domain_start()), std::min(std::max(t0, t1), curve.domain_end()));
        if (t0 > t1)
            uv.reverse();
        for (int i = 0; i < uv.cv_count(); ++i) {
            const Point p = uv.get_cv(i);
            const Vector r(p[0] - o[0], p[1] - o[1], p[2] - o[2]);
            const double x = (bb * r.dot(a) - ab * r.dot(b)) / det, y = (aa * r.dot(b) - ab * r.dot(a)) / det, w = uv.weight(i);
            uv.set_cv_4d(i, (u0 + x * (u1 - u0)) * w, (v0 + y * (v1 - v0)) * w, 0, w);
        }
        return uv;
    }
    std::vector<Point> points;
    const std::array<double, 2> period = surface_period(surface);
    const int n = 32;
    for (int i = 0; i <= n; ++i) {
        const auto [u, v] = surface.closest_parameters(curve.point_at(t0 + (t1 - t0) * i / n));
        Point uv(u, v, 0);
        for (int d = 0; d < 2 && !points.empty(); ++d)
            if (period[d] > 0.0)
                uv[d] -= std::round((uv[d] - points.back()[d]) / period[d]) * period[d];
        points.push_back(uv);
    }
    NurbsCurve uv = NurbsCurve::create_interpolated(points, CurveNurbsKnotStyle::Uniform);
    uv.set_domain(t0, t1);
    return uv;
}

Span ranged(const std::shared_ptr<const Curve>& pcurve, std::pair<double, double> domain, double t0, double t1) {
    const auto [p0, p1] = pcurve->domain();
    const auto map = [&](double t) { return p0 + (t - domain.first) / (domain.second - domain.first) * (p1 - p0); };
    return {pcurve, map(t0), map(t1), {}};
}

Span borrowed(const Curve& curve, double t0, double t1) {
    return {std::shared_ptr<const Curve>(std::shared_ptr<const Curve>(), &curve), t0, t1, {}};
}

Span pcurve_span(const BRep& brep, int edge, int face, BRepOrientation orientation) {
    for (const BRepCurveOnSurface& use : brep.m_edges[edge].pcurves)
        if (use.surface_index == brep.m_faces[face].surface_index) {
            const bool second = orientation == BRepOrientation::Reversed && use.curve_2d_index_2 >= 0;
            const auto [t0, t1] = brep.pcurve_range(use, second);
            return borrowed(brep.m_curves_2d[second ? use.curve_2d_index_2 : use.curve_2d_index], t0, t1);
        }
    return {};
}

Span edge_span(const BRep& brep, int edge) {
    const auto [t0, t1] = brep.edge_range(edge);
    return borrowed(brep.m_curves_3d[brep.m_edges[edge].curve_3d_index], t0, t1);
}

Span owned(Curve curve) {
    const auto shared = std::make_shared<const Curve>(std::move(curve));
    return {shared, shared->domain().first, shared->domain().second, {}};
}

Point Span::at(double s) const {
    const auto [lo, hi] = curve->domain();
    Point p = curve->point_at(std::clamp(t0 + s * (t1 - t0), lo, hi));
    p[0] += shift[0];
    p[1] += shift[1];
    return p;
}

Curve Span::copy() const {
    const auto [d0, d1] = curve->domain();
    const double lo = std::max(std::min(t0, t1), d0), hi = std::min(std::max(t0, t1), d1);
    if (curve->kind() != Curve::Kind::Nurbs) {
        Curve2d piece(*curve);
        piece.set_domain(lo, hi);
        if (t0 > t1)
            piece.reverse();
        return shift[0] != 0.0 || shift[1] != 0.0 ? piece.shifted(shift[0], shift[1]) : piece;
    }
    NurbsCurve piece = curve->nurbs();
    const double snap = 1e-12 * std::max(1.0, std::abs(d1 - d0));
    if (lo > d0 + snap || hi < d1 - snap)
        piece.trim(lo, hi);
    if (t0 > t1)
        piece.reverse();
    if (shift[0] != 0.0 || shift[1] != 0.0)
        for (int i = 0; i < piece.cv_count(); ++i) {
            double x, y, z, w;
            piece.get_cv_4d(i, x, y, z, w);
            piece.set_cv_4d(i, x + shift[0] * w, y + shift[1] * w, z, w);
        }
    return piece;
}

std::array<double, 2> surface_period(const Surface& s) {
    return s.kind() == Surface::Kind::Nurbs ? surface_period(s.nurbs()) : std::array<double, 2>{s.period(0), s.period(1)};
}

Span DS::edge_pcurve(const Edge& e, int local_face, BRepOrientation orientation, double t0, double t1) const {
    const Span whole = pcurve_span(brep(e.operand), e.local, local_face, orientation);
    const auto map = [&](double t) { return whole.t0 + (t - e.t0) / (e.t1 - e.t0) * (whole.t1 - whole.t0); };
    return {whole.curve, map(t0), map(t1), {}};
}

}
