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
    // On an analytic surface the point may come from any turn: first into the one the loops start in.
    Point base = uv;
    for (int d = 0; d < 2 && whole; ++d)
        if (period[d] > 0.0 && lo[d] <= hi[d])
            base[d] -= std::floor((base[d] - lo[d]) / period[d]) * period[d];
    int best = -1;
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j) {
            if ((i && !period[0]) || (j && !period[1]))
                continue;
            const Point q(base[0] + i * period[0], base[1] + j * period[1], 0);
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
                        const Point to = i == n ? b : arc.curve->point_at(arc.t0 + (arc.t1 - arc.t0) * i / n) + arc.turn;
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

int samples(const Curve& curve) {
    return std::clamp(4 * curve.nurbs().cv_count(), 8, 256);
}

bool straight(const Curve& curve) {
    return curve.kind() == Curve::Kind::Line || (curve.kind() == Curve::Kind::Nurbs && curve.nurbs().degree() == 1);
}

double parameter(const Curve& curve, const Point& p, double lo, double hi) {
    double t = curve.closest_parameter(p);
    if (const double period = curve.kind() == Curve::Kind::Nurbs ? 0.0 : curve.period(); period > 0.0) {
        t = Curve::wrapped(t, lo, period);
        // Past the piece: the end nearer around the turn.
        return t <= hi ? t : t - hi < lo + period - t ? hi : lo;
    }
    return std::clamp(t, lo, hi);
}

double true_parameter(const Curve& curve, double t) {
    switch (curve.kind()) {
    case Curve::Kind::Circle:
    case Curve::Kind::Ellipse: return Curve::angle(t, curve.domain().first, curve.domain().second);
    case Curve::Kind::Hyperbola: return curve.closest_parameter(curve.nurbs().point_at(t));
    default: return t;
    }
}

Point true_parameters(const Surface& surface, double u, double v) {
    if (surface.kind() == Surface::Kind::Nurbs || surface.kind() == Surface::Kind::Plane)
        return Point(u, v, 0);
    if (surface.kind() == Surface::Kind::Revolution || surface.kind() == Surface::Kind::Extrusion) {
        const auto [a, b] = surface.closest_parameters(surface.nurbs().point_at(u, v));
        const auto [x, y] = surface.adjusted(a, b, {u - 1.0, u + 1.0, v - 1.0, v + 1.0});
        return Point(x, y, 0);
    }
    const auto [u0, u1] = surface.domain(0);
    const auto [v0, v1] = surface.domain(1);
    const bool round = surface.kind() == Surface::Kind::Sphere || surface.kind() == Surface::Kind::Torus;
    return Point(Curve::angle(u, u0, u1), round ? Curve::angle(v, v0, v1) : v, 0);
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
    const Surface& s = brep.m_surfaces[brep.m_faces[face].surface_index];
    region.period = surface_period(s);
    region.whole = s.kind() != Surface::Kind::Nurbs;
    for (const BRepRef& wire : brep.m_faces[face].wires) {
        std::vector<Point> loop;
        std::vector<Region::Arc> arcs;
        Point last;
        for (const BRepRef& use : brep.wire_edges(wire)) {
            const Span whole = pcurve_span(brep, use.index, face, use.orientation);
            if (!whole.curve)
                continue;
            const Curve& p = *whole.curve;
            const int n = samples(p);
            const double t0 = whole.t0, span = whole.t1 - t0;
            const bool back = use.orientation == BRepOrientation::Reversed;
            // On an analytic surface each edge starts in the turn the one before it ends in.
            Vector turn(0, 0, 0);
            for (int d = 0; d < 2 && region.whole && !loop.empty(); ++d)
                if (region.period[d] > 0.0)
                    turn[d] = std::round((last[d] - p.point_at(back ? whole.t1 : t0)[d]) / region.period[d]) * region.period[d];
            for (int i = 0; i < n; ++i) {
                const double f = back ? 1.0 - double(i) / n : double(i) / n, g = back ? 1.0 - double(i + 1) / n : double(i + 1) / n;
                const Point a = p.point_at(t0 + f * span) + turn, b = p.point_at(t0 + g * span) + turn;
                double bulge = 0.0;
                if (!straight(p))
                    for (double k : {0.25, 0.5, 0.75})
                        bulge = std::max(bulge, segment_distance(p.point_at(t0 + (f + k * (g - f)) * span) + turn, a, b));
                loop.push_back(a);
                arcs.push_back({&p, t0 + f * span, t0 + g * span, bulge, turn});
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
    // BRepLib::BoundingVertex of two: the smallest ball that holds both, the larger one itself where it holds the other.
    if (real_ids.size() == 2) {
        const Vertex &large = vertices[real_ids[vertices[real_ids[0]].tolerance < vertices[real_ids[1]].tolerance ? 1 : 0]], &small = vertices[real_ids[&large == &vertices[real_ids[0]] ? 1 : 0]];
        const double apart = large.point.distance(small.point), k = apart <= large.tolerance - small.tolerance ? 0.0 : 0.5 * (1.0 - (large.tolerance - small.tolerance) / apart);
        for (int d = 0; d < 3; ++d)
            c[d] = large.point[d] + k * (small.point[d] - large.point[d]);
        tolerance = k == 0.0 ? large.tolerance : 0.5 * (large.tolerance + small.tolerance + apart);
    }
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

namespace {

NurbsCurve interpolated(const std::vector<Point>& points, double t0, double t1) {
    NurbsCurve uv = NurbsCurve::create_interpolated(points, CurveNurbsKnotStyle::Uniform);
    uv.set_domain(t0, t1);
    return uv;
}

/// The pcurve on an analytic surface, or of an analytic curve on a freeform one
Curve project_true(const Curve& curve, double t0, double t1, const Surface& surface) {
    const int n = 32;
    const std::array<double, 2> period = surface_period(surface);
    std::vector<Point> points;
    std::vector<bool> singular;
    for (int i = 0; i <= n; ++i) {
        const auto [u, v] = surface.closest_parameters(curve.point_at(t0 + (t1 - t0) * i / n));
        const std::vector<Vector> d = surface.evaluate(u, v, 1);
        points.push_back(Point(u, v, 0));
        singular.push_back(std::min(d[1].magnitude(), d[2].magnitude()) <= 1e-9 * std::max(d[1].magnitude(), d[2].magnitude()));
    }
    // At a pole or an apex the angle is the one the curve arrives at; elsewhere each sample is on the turn of the one before.
    const int regular = static_cast<int>(std::find(singular.begin(), singular.end(), false) - singular.begin());
    for (int i = regular + 1; i <= n; ++i)
        for (int d = 0; d < 2; ++d)
            if (singular[i] && d == 0)
                points[i][d] = points[i - 1][d];
            else if (period[d] > 0.0)
                points[i][d] -= std::round((points[i][d] - points[i - 1][d]) / period[d]) * period[d];
    for (int i = std::min(regular, n) - 1; i >= 0; --i)
        points[i][0] = points[i + 1][0];
    double scale = 1.0, bend = 0.0;
    for (const Point& p : points)
        scale = std::max({scale, std::abs(p[0]), std::abs(p[1])});
    for (int i = 0; i <= n; ++i)
        for (int d = 0; d < 2; ++d)
            bend = std::max(bend, std::abs(points[0][d] + (points[n][d] - points[0][d]) * i / n - points[i][d]));
    if (bend <= 1e-12 * scale && points[0].distance(points[n]) > 1e-12 * scale)
        return Curve2d::line(points[0][0], points[0][1], points[n][0], points[n][1]);
    const bool conic = curve.kind() == Curve::Kind::Circle || curve.kind() == Curve::Kind::Ellipse;
    if (surface.kind() == Surface::Kind::Plane && conic && t0 < t1 && std::abs(curve.z().dot(surface.z())) >= 1.0 - 1e-14) {
        const auto [u, v] = surface.closest_parameters(curve.origin());
        const Vector x(curve.x().dot(surface.x()), curve.x().dot(surface.y()), 0), y(curve.y().dot(surface.x()), curve.y().dot(surface.y()), 0);
        return curve.kind() == Curve::Kind::Circle ? Curve::circle(Point(u, v, 0), x, y, curve.radius(), t0, t1) : Curve::ellipse(Point(u, v, 0), x, y, curve.radius(), curve.minor(), t0, t1);
    }
    return interpolated(points, t0, t1);
}

}

/// The X direction gp_Ax2 gets from a main direction alone (DirToAx2)
static Vector canonical(const Vector& d) {
    const double x = std::abs(d[0]) < 1e-12 ? 0.0 : d[0], y = std::abs(d[1]) < 1e-12 ? 0.0 : d[1], z = std::abs(d[2]) < 1e-12 ? 0.0 : d[2];
    const double ax = std::abs(x), ay = std::abs(y), az = std::abs(z);
    Vector r = ax == 0.0 || (ax < ay && ax < az) ? Vector(0, -z, y) : ay == 0.0 || (ay < ax && ay < az) ? Vector(-z, 0, x) : Vector(-y, x, 0);
    r.normalize_self();
    return r;
}

bool conic_origin(const Point& o, Vector n, int kind, const Point& c, const Vector& axis, double size, const Point& fallback, Point& origin) {
    n.normalize_self();
    Vector x;
    Point center;
    double radius = 0.0;
    if (kind == 2) {
        const double d = Vector(c[0] - o[0], c[1] - o[1], c[2] - o[2]).dot(n);
        center = Point(c[0] - n[0] * d, c[1] - n[1] * d, c[2] - n[2] * d);
        radius = std::sqrt(std::max(size * size - d * d, 0.0));
        x = canonical(n);
    } else if (kind == 3 || kind == 4) {
        const double cosine = axis.dot(n);
        if (std::abs(cosine) < 1e-9)
            return false;
        const double k = Vector(o[0] - c[0], o[1] - c[1], o[2] - c[2]).dot(n) / cosine;
        center = Point(c[0] + axis[0] * k, c[1] + axis[1] * k, c[2] + axis[2] * k);
        if (axis.cross(n).magnitude() < 1e-9) {
            radius = center.distance(fallback);
            x = canonical(axis);
        } else if (kind == 3) {
            radius = size / std::abs(cosine);
            x = axis - n * cosine;
            x.normalize_self();
        } else
            return false;
    } else
        return false;
    origin = Point(center[0] + x[0] * radius, center[1] + x[1] * radius, center[2] + x[2] * radius);
    return true;
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

Curve project(const Curve& whole, double t0, double t1, const Surface& form) {
    const bool flat = form.kind() == Surface::Kind::Plane;
    if (whole.kind() != Curve::Kind::Nurbs || (form.kind() != Surface::Kind::Nurbs && !flat))
        return project_true(whole, t0, t1, form);
    const NurbsCurve& curve = whole.nurbs();
    Point o = form.origin();
    Vector a = form.x(), b = form.y();
    if (flat || affine_patch(form.nurbs(), o, a, b)) {
        const double aa = a.dot(a), ab = a.dot(b), bb = b.dot(b), det = aa * bb - ab * ab;
        const auto [u0, u1] = flat ? std::pair{0.0, 1.0} : form.nurbs().domain(0);
        const auto [v0, v1] = flat ? std::pair{0.0, 1.0} : form.nurbs().domain(1);
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
    const NurbsSurface& surface = form.nurbs();
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
    return interpolated(points, t0, t1);
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
    const double t = t0 + s * (t1 - t0);
    Point p = curve->point_at(curve->kind() == Curve::Kind::Nurbs ? std::clamp(t, lo, hi) : t);
    p[0] += shift[0];
    p[1] += shift[1];
    return p;
}

Curve Span::copy() const {
    const auto [d0, d1] = curve->domain();
    const double lo = std::max(std::min(t0, t1), d0), hi = std::min(std::max(t0, t1), d1);
    if (curve->kind() != Curve::Kind::Nurbs) {
        Curve2d piece(*curve);
        piece.set_domain(std::min(t0, t1), std::max(t0, t1));
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
