#include "bop_ds.h"
#include <algorithm>
#include <cmath>

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

void Region::add_loop(std::vector<Point> loop) {
    for (const Point& p : loop)
        for (int d = 0; d < 2; ++d) {
            lo[d] = std::min(lo[d], p[d]);
            hi[d] = std::max(hi[d], p[d]);
        }
    loops.push_back(std::move(loop));
}

static double segment_distance(const Point& p, const Point& a, const Point& b) {
    const double dx = b[0] - a[0], dy = b[1] - a[1], len = dx * dx + dy * dy;
    double s = len > 0 ? ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len : 0.0;
    s = std::clamp(s, 0.0, 1.0);
    return std::hypot(p[0] - a[0] - s * dx, p[1] - a[1] - s * dy);
}

int Region::state(const Point& uv) const {
    int best = -1;
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j) {
            if ((i && !period[0]) || (j && !period[1]))
                continue;
            const Point q(uv[0] + i * period[0], uv[1] + j * period[1], 0);
            if (q[0] < lo[0] - eps || q[0] > hi[0] + eps || q[1] < lo[1] - eps || q[1] > hi[1] + eps)
                continue;
            bool inside = false, on = false;
            for (const auto& loop : loops)
                for (size_t k = 0; k < loop.size() && !on; ++k) {
                    const Point& a = loop[k];
                    const Point& b = loop[(k + 1) % loop.size()];
                    if (segment_distance(q, a, b) <= eps)
                        on = true;
                    else if ((a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0] + (q[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]))
                        inside = !inside;
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
    double widest = 0.0;
    for (double f : {0.5, 0.37, 0.63, 0.23, 0.77, 0.11, 0.89, 0.03, 0.97}) {
        const double v = lo[1] + f * (hi[1] - lo[1]);
        std::vector<double> xs;
        for (const auto& loop : loops)
            for (size_t k = 0; k < loop.size(); ++k) {
                const Point& a = loop[k];
                const Point& b = loop[(k + 1) % loop.size()];
                if ((a[1] > v) != (b[1] > v))
                    xs.push_back(a[0] + (v - a[1]) / (b[1] - a[1]) * (b[0] - a[0]));
            }
        std::sort(xs.begin(), xs.end());
        for (size_t k = 0; k + 1 < xs.size(); k += 2)
            if (xs[k + 1] - xs[k] > widest) {
                const Point mid(0.5 * (xs[k] + xs[k + 1]), v, 0);
                if (state(mid) == 1) {
                    widest = xs[k + 1] - xs[k];
                    uv = mid;
                }
            }
        if (widest > 0.2 * (hi[0] - lo[0]))
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
    const NurbsSurface& s = brep.m_surfaces[brep.m_faces[face].surface_index];
    region.period = surface_period(s);
    for (const BRepRef& wire : brep.m_faces[face].wires) {
        std::vector<Point> loop;
        for (const BRepRef& use : brep.wire_edges(wire)) {
            const int index = brep.pcurve_index(use.index, face, use.orientation);
            if (index < 0)
                continue;
            const NurbsCurve& p = brep.m_curves_2d[index];
            const int n = samples(p);
            for (int i = 0; i < n; ++i) {
                const double f = use.orientation == BRepOrientation::Reversed ? 1.0 - double(i) / n : double(i) / n;
                loop.push_back(p.point_at(p.domain_start() + f * (p.domain_end() - p.domain_start())));
            }
        }
        if (loop.size() > 2)
            region.add_loop(std::move(loop));
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
        NurbsCurve uv = trimmed(curve, curve.domain(), t0, t1);
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

NurbsCurve trimmed(const NurbsCurve& curve, std::pair<double, double> domain, double t0, double t1) {
    const auto map = [&](double t) {
        return curve.domain_start() + (t - domain.first) / (domain.second - domain.first) * (curve.domain_end() - curve.domain_start());
    };
    double s0 = map(t0), s1 = map(t1);
    const bool flip = s0 > s1;
    if (flip)
        std::swap(s0, s1);
    NurbsCurve piece = curve;
    const double span = curve.domain_end() - curve.domain_start();
    const double snap = 1e-12 * std::max(1.0, std::abs(span));
    if (s0 > curve.domain_start() + snap || s1 < curve.domain_end() - snap)
        piece.trim(std::max(s0, curve.domain_start()), std::min(s1, curve.domain_end()));
    if (flip)
        piece.reverse();
    return piece;
}

}
