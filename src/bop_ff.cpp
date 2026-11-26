#include "bop_ff.h"
#include "bop_ds.h"
#include "closest.h"
#include "curve_intersection.h"
#include "intersection_analytic.h"
#include "nurbs_coincidence.h"
#include "nurbs_implicit.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace session_cpp::bop_ff {
namespace {

using X = std::array<double, 4>;
using P3 = session_cpp::Point;
constexpr double infinity = std::numeric_limits<double>::infinity();

struct Side {
    const NurbsSurface* full = nullptr;
    NurbsSurface surface;
    Surface form; // an analytic surface over the box of the face, `surface` its NURBS
    bool analytic = false;
    std::array<double, 4> box{};
    std::array<bool, 2> closed{};
    nurbs_implicit::Implicit implicit;
    double size = 0.0;
    /// The true parameters under (u, v) of `surface`
    P3 true_uv(double u, double v) const { return analytic ? bop::true_parameters(form, u, v) : P3(u, v, 0); }
    /// The point at true parameters
    P3 at(const P3& uv) const { return analytic ? form.point_at(uv[0], uv[1]) : full->point_at(uv[0], uv[1]); }
};

struct Jet {
    xyz p, du, dv;
};

struct Node {
    X x;
    xyz p;
};

struct Context {
    const Side& a;
    const Side& b;
    double tol, size, eps, h0, hmin, hmax;
    double flat = 1e-6;
    std::vector<xyz> through{};
};

struct Constraint {
    int k = -2; // -2 none, -1 plane (p - origin).dir = h, else parameter k held at value
    double value = 0.0;
    xyz origin{}, dir{};
    double h = 0.0;
};

Jet jet(const NurbsSurface& s, double u, double v) {
    std::array<xyz, 6> d{};
    s.jet(u, v, 1, d);
    return {d[0], d[2], d[1]}; // jet writes S, Sv, Su
}

xyz unit(const xyz& v) {
    const double l = v.length();
    return l > 0 ? v / l : v;
}

// The diagonal of the control net's box.
double diagonal(const NurbsSurface& s) {
    std::array<double, 6> hull = {infinity, -infinity, infinity, -infinity, infinity, -infinity};
    for (int i = 0; i < s.cv_count(0); ++i)
        for (int j = 0; j < s.cv_count(1); ++j) {
            const P3 q = s.get_cv(i, j);
            for (int k = 0; k < 3; ++k) {
                hull[2 * k] = std::min(hull[2 * k], q[k]);
                hull[2 * k + 1] = std::max(hull[2 * k + 1], q[k]);
            }
        }
    return std::hypot(hull[1] - hull[0], hull[3] - hull[2], hull[5] - hull[4]);
}

// The range of one coordinate of a pcurve: its control points for a polyline, else samples refined at both extremes.
std::pair<double, double> extent(const NurbsCurve& c, int d) {
    std::pair<double, double> range = {infinity, -infinity};
    if (c.degree() == 1) {
        for (int i = 0; i < c.cv_count(); ++i)
            range = {std::min(range.first, c.get_cv(i)[d]), std::max(range.second, c.get_cv(i)[d])};
        return range;
    }
    const int n = 8 * c.span_count();
    const double t0 = c.domain_start(), dt = (c.domain_end() - t0) / n, golden = 0.5 * (std::sqrt(5.0) - 1.0);
    for (double sign : {-1.0, 1.0}) {
        int at = 0;
        for (int i = 1; i <= n; ++i)
            if (sign * c.point_at(t0 + i * dt)[d] > sign * c.point_at(t0 + at * dt)[d])
                at = i;
        double lo = t0 + std::max(at - 1, 0) * dt, hi = t0 + std::min(at + 1, n) * dt;
        for (int it = 0; it < 40; ++it) {
            const double m1 = hi - golden * (hi - lo), m2 = lo + golden * (hi - lo);
            const bool left = sign * c.point_at(m1)[d] > sign * c.point_at(m2)[d];
            (left ? hi : lo) = left ? m2 : m1;
        }
        const double value = c.point_at(0.5 * (lo + hi))[d];
        range = {std::min(range.first, value), std::max(range.second, value)};
    }
    return range;
}

// BRepTools::UVBounds: the box of the face's pcurves, the whole domain where they leave it (a face across a seam). Not grown as
// CorrectSurfaceBoundaries grows it: a section then ends on the edge it meets, not a tolerance past it.
std::array<double, 4> uv_box(const BRep& brep, int face) {
    const BRepFace& f = brep.m_faces[face];
    if (const Surface& form = brep.m_surfaces[f.surface_index]; form.kind() != Surface::Kind::Nurbs) {
        // An analytic surface is whole: the box of the pcurves over their ranges, wherever it lies; an angle over no more than its turn.
        std::array<double, 4> box = {infinity, -infinity, infinity, -infinity};
        for (const BRepRef& w : f.wires)
            for (const BRepRef& e : brep.m_wires[w.index].edges)
                for (BRepOrientation o : {BRepOrientation::Forward, BRepOrientation::Reversed}) {
                    const bop::Span p = bop::pcurve_span(brep, e.index, face, o);
                    for (int d = 0; d < 2 && p.curve; ++d) {
                        const std::pair<double, double> range = extent(p.copy().nurbs(), d);
                        box[2 * d] = std::min(box[2 * d], range.first), box[2 * d + 1] = std::max(box[2 * d + 1], range.second);
                    }
                }
        for (int d = 0; d < 2; ++d) {
            if (!(box[2 * d + 1] - box[2 * d] > 1e-12))
                box[2 * d] = form.domain(d).first, box[2 * d + 1] = form.domain(d).second;
            if (form.period(d) > 0.0 && box[2 * d + 1] - box[2 * d] > form.period(d) * (1.0 - 1e-9))
                box[2 * d + 1] = box[2 * d] + form.period(d);
        }
        return box;
    }
    const NurbsSurface& s = brep.m_surfaces[f.surface_index].nurbs();
    std::array<double, 4> box = {s.domain(0).first, s.domain(0).second, s.domain(1).first, s.domain(1).second};
    std::array<double, 4> found = {infinity, -infinity, infinity, -infinity};
    for (const BRepRef& w : f.wires)
        for (const BRepRef& e : brep.m_wires[w.index].edges)
            for (const BRepCurveOnSurface& on : brep.m_edges[e.index].pcurves)
                if (on.surface_index == f.surface_index)
                    for (int index : {on.curve_2d_index, on.curve_2d_index_2})
                        if (index >= 0)
                            for (int d = 0; d < 2; ++d) {
                                const std::pair<double, double> range = extent(brep.m_curves_2d[index].nurbs(), d);
                                found[2 * d] = std::min(found[2 * d], range.first);
                                found[2 * d + 1] = std::max(found[2 * d + 1], range.second);
                            }
    for (int d = 0; d < 2; ++d) {
        const double span = box[2 * d + 1] - box[2 * d], lo = found[2 * d], hi = found[2 * d + 1];
        if (!(hi - lo > 1e-9 * span) || lo < box[2 * d] - 0.01 * span || hi > box[2 * d + 1] + 0.01 * span)
            continue;
        box[2 * d] = std::max(box[2 * d], lo);
        box[2 * d + 1] = std::min(box[2 * d + 1], hi);
    }
    return box;
}

Side make_side(const BRep& brep, int face, double tol) {
    Side side;
    const Surface& form = brep.m_surfaces[brep.m_faces[face].surface_index];
    side.full = &form.nurbs();
    side.surface = *side.full;
    const std::array<double, 4> box = uv_box(brep, face);
    side.analytic = form.kind() != Surface::Kind::Nurbs;
    if (side.analytic) {
        side.form = form;
        for (int d = 0; d < 2; ++d) {
            side.form.set_domain(d, box[2 * d], box[2 * d + 1]);
            side.closed[d] = form.period(d) > 0.0 && box[2 * d + 1] - box[2 * d] >= form.period(d) * (1.0 - 1e-9);
        }
        side.surface = side.form.nurbs();
        side.box = box;
    }
    for (int d = 0; d < 2 && !side.analytic; ++d) {
        const std::pair<double, double> domain = side.full->domain(d);
        const bool whole = box[2 * d] <= domain.first && box[2 * d + 1] >= domain.second;
        NurbsSurface trimmed = side.surface;
        if (!whole && trimmed.trim(d, {box[2 * d], box[2 * d + 1]}))
            side.surface = trimmed;
        side.box[2 * d] = side.surface.domain(d).first;
        side.box[2 * d + 1] = side.surface.domain(d).second;
        side.closed[d] = whole && side.full->is_closed(d);
    }
    side.implicit = nurbs_implicit::recognize(side.surface, tol);
    side.size = diagonal(side.surface);
    return side;
}

// The distance from p to the plane, sphere, cylinder, cone or torus.
double implicit_gap(const nurbs_implicit::Implicit& f, const xyz& p) {
    const xyz axis{f.axis[0], f.axis[1], f.axis[2]};
    const xyz d = p - xyz{f.point[0], f.point[1], f.point[2]};
    const double z = d.dot(axis), rho = (d - axis * z).length();
    switch (f.kind) {
    case 1: return std::abs(z);
    case 2: return std::abs(d.length() - f.radius);
    case 3: return std::abs(rho - f.radius);
    case 4: return d.length() * std::abs(std::sin(std::min(std::atan2(rho, std::abs(z)) - f.radius, M_PI / 2)));
    default: return std::abs(std::hypot(rho - f.radius, z) - f.minor);
    }
}

// Samples of a box of a against b: how many lie within tol of b inside its box, -1 once one lies farther.
int covered(const Side& a, const std::array<double, 4>& box, const Side& b, double tol) {
    const int n = 9;
    int inside = 0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const int si = (i + n / 2) % n, sj = (j + n / 2) % n;
            const xyz p = a.surface.point(box[0] + (si + 0.5) / n * (box[1] - box[0]), box[2] + (sj + 0.5) / n * (box[3] - box[2]));
            if (b.implicit.kind) {
                if (implicit_gap(b.implicit, p) > tol)
                    return -1;
                ++inside;
                continue;
            }
            const auto [u, v, gap] = Closest::surface_point(b.surface, P3(p));
            const double eu = 1e-9 * (b.box[1] - b.box[0]), ev = 1e-9 * (b.box[3] - b.box[2]);
            if (u <= b.box[0] + eu || u >= b.box[1] - eu || v <= b.box[2] + ev || v >= b.box[3] - ev)
                continue;
            if (gap > tol)
                return -1;
            ++inside;
        }
    return inside;
}

// IntPatch TangentFaces: one surface within tol of the other wherever the faces' boxes lie over each other.
bool same_domain(const Side& a, const Side& b, double tol) {
    nurbs_coincidence::ParameterMap mapping;
    if (nurbs_coincidence::same_parameters(*a.full, *b.full, tol) || nurbs_coincidence::parameter_mapping(*a.full, *b.full, tol, mapping)
        || nurbs_coincidence::common_basis_agreement(*a.full, *b.full, tol) || nurbs_coincidence::coplanar(*a.full, *b.full, tol))
        return true;
    if (a.implicit.kind && b.implicit.kind)
        return a.implicit.kind == b.implicit.kind && covered(a, a.box, b, tol) > 0;
    const int ab = covered(a, a.box, b, tol), ba = ab < 0 ? -1 : covered(b, b.box, a, tol);
    return ab >= 0 && ba >= 0 && std::max(ab, ba) >= 9;
}

// Into the boxes; around the seam of a closed face when wrap is set.
void clamp(const Context& c, X& x, bool wrap = false) {
    for (int k = 0; k < 4; ++k) {
        const Side& s = k < 2 ? c.a : c.b;
        const double lo = s.box[2 * (k % 2)], hi = s.box[2 * (k % 2) + 1];
        x[k] = wrap && s.closed[k % 2] ? lo + std::fmod(std::fmod(x[k] - lo, hi - lo) + (hi - lo), hi - lo) : std::clamp(x[k], lo, hi);
    }
}

// Midway between the two surfaces, and how far that is from each.
xyz point(const Context& c, const X& x) {
    return (c.a.surface.point(x[0], x[1]) + c.b.surface.point(x[2], x[3])) * 0.5;
}

double half_gap(const Context& c, const X& x) {
    return 0.5 * (c.a.surface.point(x[0], x[1]) - c.b.surface.point(x[2], x[3])).length();
}

// The length on its surface of a step dk of parameter k.
double travel(const Context& c, const X& x, int k, double dk) {
    const Jet j = k < 2 ? jet(c.a.surface, x[0], x[1]) : jet(c.b.surface, x[2], x[3]);
    return (k % 2 ? j.dv : j.du).length() * std::abs(dk);
}

// The unit section direction na x nb and the sine of the angle between the surfaces.
xyz tangent(const Context& c, const X& x, double& sine) {
    const Jet ja = jet(c.a.surface, x[0], x[1]), jb = jet(c.b.surface, x[2], x[3]);
    const xyz t = unit(ja.du.cross(ja.dv)).cross(unit(jb.du.cross(jb.dv)));
    sine = t.length();
    return unit(t);
}

double residual(const Context& c, const X& x, const Constraint& con, std::array<double, 4>& r, std::array<X, 4>& J) {
    const Jet ja = jet(c.a.surface, x[0], x[1]), jb = jet(c.b.surface, x[2], x[3]);
    const xyz f = ja.p - jb.p;
    const std::array<xyz, 4> columns = {ja.du, ja.dv, -jb.du, -jb.dv};
    for (int i = 0; i < 3; ++i) {
        r[i] = f[i];
        for (int j = 0; j < 4; ++j)
            J[i][j] = columns[j][i];
    }
    r[3] = 0.0;
    J[3] = {0, 0, 0, 0};
    if (con.k == -1) {
        r[3] = (ja.p - con.origin).dot(con.dir) - con.h;
        J[3] = {ja.du.dot(con.dir), ja.dv.dot(con.dir), 0, 0};
    } else if (con.k >= 0) {
        const double speed = std::max(columns[con.k].length(), 1e-12 * c.size);
        r[3] = (x[con.k] - con.value) * speed;
        J[3][con.k] = speed;
    }
    return f.length();
}

double bound(const Context& c, int k, int end) {
    return (k < 2 ? c.a : c.b).box[2 * (k % 2) + end];
}

bool solve(std::array<std::array<double, 5>, 4>& m, X& out) {
    for (int i = 0; i < 4; ++i) {
        int pivot = i;
        for (int k = i + 1; k < 4; ++k)
            if (std::abs(m[k][i]) > std::abs(m[pivot][i]))
                pivot = k;
        if (!(std::abs(m[pivot][i]) > 1e-300))
            return false;
        std::swap(m[i], m[pivot]);
        for (int k = i + 1; k < 4; ++k) {
            const double f = m[k][i] / m[i][i];
            for (int j = i; j < 5; ++j)
                m[k][j] -= f * m[i][j];
        }
    }
    for (int i = 3; i >= 0; --i) {
        double s = m[i][4];
        for (int j = i + 1; j < 4; ++j)
            s -= m[i][j] * out[j];
        out[i] = s / m[i][i];
    }
    return std::isfinite(out[0] + out[1] + out[2] + out[3]);
}

// Levenberg-Marquardt on Sa(u, v) = Sb(s, t) and the constraint; gap is the distance left between the surfaces.
bool correct(const Context& c, X& x, const Constraint& con, double& gap, bool wrap = false) {
    std::array<double, 4> r{};
    std::array<X, 4> J{};
    gap = residual(c, x, con, r, J);
    double norm = r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3], lambda = 1e-9;
    for (int it = 0; it < 60 && norm > c.eps * c.eps; ++it) {
        // A parameter on its box bound that the step would push out stays there (active set).
        std::array<bool, 4> held{};
        X dx{};
        bool solved = false;
        for (int pass = 0; pass < 4; ++pass) {
            std::array<std::array<double, 5>, 4> m{};
            double scale = 0.0;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j)
                    for (int k = 0; k < 4; ++k)
                        m[i][j] += held[i] || held[j] ? 0.0 : J[k][i] * J[k][j];
                for (int k = 0; k < 4; ++k)
                    m[i][4] -= held[i] ? 0.0 : J[k][i] * r[k];
                scale = std::max(scale, m[i][i]);
            }
            for (int i = 0; i < 4; ++i)
                m[i][i] = held[i] ? 1.0 : m[i][i] + lambda * (m[i][i] + 1e-12 * scale);
            if (!(solved = solve(m, dx)))
                break;
            bool more = false;
            for (int k = 0; k < 4; ++k)
                if (!held[k] && !(wrap && (k < 2 ? c.a : c.b).closed[k % 2])
                    && ((x[k] <= bound(c, k, 0) && dx[k] < 0) || (x[k] >= bound(c, k, 1) && dx[k] > 0)))
                    held[k] = more = true;
            if (!more)
                break;
        }
        if (!solved) {
            lambda *= 10;
            continue;
        }
        X y = {x[0] + dx[0], x[1] + dx[1], x[2] + dx[2], x[3] + dx[3]};
        clamp(c, y, wrap);
        std::array<double, 4> r2{};
        std::array<X, 4> J2{};
        const double gap2 = residual(c, y, con, r2, J2);
        const double norm2 = r2[0] * r2[0] + r2[1] * r2[1] + r2[2] * r2[2] + r2[3] * r2[3];
        if (norm2 < norm) {
            bool still = true;
            for (int k = 0; k < 4; ++k) {
                const Side& s = k < 2 ? c.a : c.b;
                still = still && std::abs(y[k] - x[k]) <= 1e-15 * (s.box[2 * (k % 2) + 1] - s.box[2 * (k % 2)]);
            }
            x = y, r = r2, J = J2, norm = norm2, gap = gap2;
            lambda = std::max(lambda * 0.1, 1e-15);
            if (still)
                break;
        } else if ((lambda *= 10) > 1e10) {
            break;
        }
    }
    return gap <= c.tol && std::abs(r[3]) <= c.tol;
}

// The parameter steps that move both surfaces by the 3D step, first order.
X predict(const Context& c, const X& x, const xyz& step) {
    X y = x;
    for (int s = 0; s < 2; ++s) {
        const Jet j = s ? jet(c.b.surface, x[2], x[3]) : jet(c.a.surface, x[0], x[1]);
        const double e = j.du.dot(j.du), f = j.du.dot(j.dv), g = j.dv.dot(j.dv), det = e * g - f * f;
        if (!(det > 1e-30 * e * g))
            continue;
        const double p = j.du.dot(step), q = j.dv.dot(step);
        y[2 * s] += (g * p - f * q) / det;
        y[2 * s + 1] += (e * q - f * p) / det;
    }
    return y;
}


// The first box bound the straight move from x to y crosses: its fraction of the move, or 2 with k = -1.
double crossing(const Context& c, const X& x, const X& y, int& k, double& value) {
    double first = 2.0;
    k = -1;
    for (int i = 0; i < 4; ++i)
        for (int end = 0; end < 2; ++end) {
            const double b = bound(c, i, end);
            if (end ? y[i] > b : y[i] < b) {
                const double f = (b - x[i]) / (y[i] - x[i]);
                if (f < first)
                    first = std::max(f, 0.0), k = i, value = b;
            }
        }
    return first;
}

// The bound y reached from x, -1 when it reached none.
int touched(const Context& c, const X& x, const X& y) {
    for (int k = 0; k < 4; ++k)
        for (int end = 0; end < 2; ++end) {
            const double b = bound(c, k, end), e = 1e-12 * std::abs(bound(c, k, 1) - bound(c, k, 0));
            if (std::abs(y[k] - b) <= e && std::abs(x[k] - b) > e)
                return k;
        }
    return -1;
}

enum class Stop { Stuck, Closed, Exit };

Node midpoint(const Context& c, const Node& a, const Node& b);

// IntWalk_PWalking: march from nodes.back() along dir until a box bound, back to loop's start, or no step converges.
Stop walk(const Context& c, std::vector<Node>& nodes, xyz dir, const Node* loop, int& exit, xyz& last) {
    double h = c.h0;
    const xyz start_dir = dir;
    while (nodes.size() < 20000) {
        const Node cur = nodes.back();
        double sine = 0.0;
        xyz t = tangent(c, cur.x, sine);
        const bool against = t.dot(dir) < 0;
        if (sine < c.flat)
            t = dir;
        else if (against)
            t = -t;
        bool moved = false;
        while (!moved && h >= c.hmin) {
            X guess = predict(c, cur.x, t * h);
            // A step along a bound stays on it: only one that heads out of the box, or at any angle over a seam, ends the line there.
            const auto heading = [&](int i) { return (i < 2 ? c.a : c.b).closed[i % 2] ? 1e-6 : 1e-2; };
            for (int i = 0; i < 4; ++i)
                if ((guess[i] < bound(c, i, 0) || guess[i] > bound(c, i, 1)) && travel(c, cur.x, i, guess[i] - cur.x[i]) <= heading(i) * h)
                    guess[i] = std::clamp(guess[i], bound(c, i, 0), bound(c, i, 1));
            // The cross product of the normals keeps its sense along a branch of the section and has the other sense on the branch that passes it where the surfaces all but touch, with no point of the section between the two. A step that landed there is cut back to the last point of its own branch, found by halving, and the section turns from it; where the two meet, the surfaces touch and the section runs on through. 0 for a step along its branch, 1 for one to retry shorter, 2 where the walk moved to the cut.
            const auto cut_back = [&](const X& y, const xyz& p) {
                double s2 = 0.0, sm = 0.0;
                const xyz t2 = tangent(c, y, s2);
                if (sine < c.flat || s2 < c.flat || (t2.dot(t) < 0) == against)
                    return 0;
                Node own = cur, other{y, p};
                bool apart = true;
                for (int it = 0; it < 48 && apart && (other.p - own.p).length() > c.eps; ++it) {
                    const Node mid = midpoint(c, own, other);
                    const xyz tm = tangent(c, mid.x, sm);
                    const bool on = half_gap(c, mid.x) <= 1e3 * c.eps;
                    apart = !on || sm >= c.flat;
                    (on && (tm.dot(t) < 0) == against ? own : other) = mid;
                }
                if (!apart)
                    return 0;
                if ((own.p - cur.p).length() <= c.hmin)
                    return 1;
                const xyz to = tangent(c, own.x, sm);
                nodes.push_back(own);
                dir = against ? -to : to, last = unit(own.p - cur.p), moved = true;
                h = std::max(c.hmin, std::min(h, (other.p - own.p).length()));
                return 2;
            };
            int k = -1;
            double value = 0.0, gap = 0.0;
            const double f = crossing(c, cur.x, guess, k, value);
            if (k >= 0) {
                X y = {cur.x[0] + f * (guess[0] - cur.x[0]), cur.x[1] + f * (guess[1] - cur.x[1]),
                       cur.x[2] + f * (guess[2] - cur.x[2]), cur.x[3] + f * (guess[3] - cur.x[3])};
                clamp(c, y);
                if (correct(c, y, {k, value}, gap)) {
                    // Next to a second bound the one the section crosses more squarely places the end: a bound it only touches leaves the end loose along it.
                    const auto pace = [&](int i) { return travel(c, y, i, predict(c, y, t)[i] - y[i]); };
                    for (int i = 0; i < 4; ++i)
                        for (int end = 0; end < 2; ++end) {
                            X z = y;
                            z[i] = bound(c, i, end);
                            if (i != k && travel(c, y, i, z[i] - y[i]) <= std::sqrt(c.tol * c.size) && pace(i) > pace(k) && correct(c, z, {i, z[i]}, gap))
                                y = z, k = i;
                        }
                    const xyz p = point(c, y), d = p - cur.p;
                    if (const int cut = cut_back(y, p)) {
                        h *= cut == 1 ? 0.5 : 1.0;
                        continue;
                    }
                    if (d.dot(t) > 0.5 * d.length() && d.length() <= 2 * h && d.length() > 0) {
                        nodes.push_back({y, p});
                        exit = k, last = unit(d);
                        return Stop::Exit;
                    }
                }
                h *= 0.5;
                continue;
            }
            X y = guess;
            if (!correct(c, y, {-1, 0.0, cur.p, t, h}, gap)) {
                h *= 0.5;
                continue;
            }
            const xyz p = point(c, y), d = p - cur.p;
            const double len = d.length();
            if (!(len > 0) || d.dot(t) < 0.98 * len) {
                h *= 0.5;
                continue;
            }
            if (const int cut = cut_back(y, p)) {
                h *= cut == 1 ? 0.5 : 1.0;
                continue;
            }
            double s2 = 0.0;
            xyz t2 = tangent(c, y, s2);
            if (s2 >= c.flat) {
                if (t2.dot(t) < 0)
                    t2 = -t2;
                if (t2.dot(t) < 0.98) {
                    h *= 0.5;
                    continue;
                }
            } else {
                t2 = d / len;
            }
            if (loop && nodes.size() > 2 && start_dir.dot(d) > 0) {
                const double along = (loop->p - cur.p).dot(d) / (len * len);
                if (along >= 0 && along <= 1 && (cur.p + d * along - loop->p).length() <= 0.05 * len + 2 * c.tol) {
                    nodes.push_back(*loop);
                    return Stop::Closed;
                }
            }
            nodes.push_back({y, p});
            dir = t2, last = d / len, moved = true;
            const int kb = touched(c, cur.x, y);
            if (kb >= 0 && travel(c, cur.x, kb, y[kb] - cur.x[kb]) > heading(kb) * len) {
                exit = kb;
                return Stop::Exit;
            }
            if (t2.dot(t) > 0.995)
                h = std::min(1.5 * h, c.hmax);
        }
        if (!moved)
            return Stop::Stuck;
    }
    return Stop::Stuck;
}

double segment_gap(const xyz& p, const xyz& a, const xyz& b) {
    const xyz ab = b - a;
    const double l2 = ab.dot(ab);
    const double f = l2 > 0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
    return (a + ab * f - p).length();
}

bool on_lines(const Context& c, const std::vector<std::vector<Node>>& lines, const xyz& p, double reach = infinity) {
    for (const std::vector<Node>& line : lines)
        for (size_t i = 1; i < line.size(); ++i)
            if (segment_gap(p, line[i - 1].p, line[i].p) <= 0.05 * std::min((line[i].p - line[i - 1].p).length(), reach) + 2 * c.tol)
                return true;
    return false;
}

// Seeds, IntPolyh in short: on a grid of each face, every grid line against the other surface near the cell holding its point,
// and every cell centre to its nearest point of the other surface.
std::vector<std::pair<Node, double>> seeds(const Context& c) {
    struct Grid {
        std::array<int, 2> n;
        std::vector<std::array<double, 6>> boxes;
        std::vector<X> centres;
    };
    std::array<Grid, 2> grids;
    const auto line = [](const Side& side, int d, int i, int n) { return side.box[2 * d] + i * (side.box[2 * d + 1] - side.box[2 * d]) / n; };
    for (int s = 0; s < 2; ++s) {
        const Side& side = s ? c.b : c.a;
        Grid& g = grids[s];
        for (int d = 0; d < 2; ++d)
            g.n[d] = std::clamp(2 * side.surface.span_count(d) + 4, 6, 16);
        for (int i = 0; i < g.n[0]; ++i)
            for (int j = 0; j < g.n[1]; ++j) {
                const double u0 = line(side, 0, i, g.n[0]), u1 = line(side, 0, i + 1, g.n[0]);
                const double v0 = line(side, 1, j, g.n[1]), v1 = line(side, 1, j + 1, g.n[1]);
                std::array<double, 6> box = {infinity, -infinity, infinity, -infinity, infinity, -infinity};
                for (int a = 0; a < 3; ++a)
                    for (int b = 0; b < 3; ++b) {
                        const xyz p = side.surface.point(u0 + 0.5 * a * (u1 - u0), v0 + 0.5 * b * (v1 - v0));
                        for (int k = 0; k < 3; ++k)
                            box[2 * k] = std::min(box[2 * k], p[k]), box[2 * k + 1] = std::max(box[2 * k + 1], p[k]);
                    }
                const double grow = 0.25 * std::hypot(box[1] - box[0], box[3] - box[2], box[5] - box[4]) + c.tol;
                for (int k = 0; k < 3; ++k)
                    box[2 * k] -= grow, box[2 * k + 1] += grow;
                g.boxes.push_back(box);
                g.centres.push_back({0.5 * (u0 + u1), 0.5 * (v0 + v1), 0, 0});
            }
    }
    // The centre of the cell of the other grid whose box holds p, nearest to p.
    const auto near = [&](int s, const xyz& p, X& uv) {
        double best = infinity;
        for (size_t i = 0; i < grids[s].boxes.size(); ++i) {
            const std::array<double, 6>& b = grids[s].boxes[i];
            double d2 = 0.0;
            bool inside = true;
            for (int k = 0; k < 3; ++k) {
                inside = inside && b[2 * k] <= p[k] && p[k] <= b[2 * k + 1];
                const double m = p[k] - 0.5 * (b[2 * k] + b[2 * k + 1]);
                d2 += m * m;
            }
            if (inside && d2 < best)
                best = d2, uv = grids[s].centres[i];
        }
        return best < infinity;
    };
    std::vector<std::pair<Node, double>> found;
    const auto attempt = [&](int s, const X& own, const Constraint& con) {
        const Side& side = s ? c.b : c.a;
        X other{};
        if (!near(1 - s, side.surface.point(own[0], own[1]), other))
            return;
        X x = s ? X{other[0], other[1], own[0], own[1]} : X{own[0], own[1], other[0], other[1]};
        double gap = 0.0, sine = 0.0;
        if (!correct(c, x, con, gap, true))
            return;
        const xyz p = point(c, x);
        for (const auto& [n, _] : found)
            if ((n.p - p).length() <= 1e-9 * c.size)
                return;
        tangent(c, x, sine);
        found.push_back({{x, p}, sine});
    };
    for (int s = 0; s < 2; ++s) {
        const Side& side = s ? c.b : c.a;
        const Grid& g = grids[s];
        for (const X& centre : g.centres)
            attempt(s, centre, {});
        for (int d = 0; d < 2; ++d)
            for (int i = 0; i <= g.n[d]; ++i)
                for (int j = 0; j < g.n[1 - d]; ++j) {
                    X own{};
                    own[d] = line(side, d, i, g.n[d]);
                    own[1 - d] = line(side, 1 - d, j, g.n[1 - d]) + 0.5 * (side.box[3 - 2 * d] - side.box[2 - 2 * d]) / g.n[1 - d];
                    attempt(s, own, {2 * s + d, own[d]});
                }
    }
    std::stable_sort(found.begin(), found.end(), [](const auto& l, const auto& r) { return l.second > r.second; });
    return found;
}

// SMLib's classes of a tangent point by the second fundamental forms: over the common tangent plane the gap between the surfaces
// is half of D11 x x + 2 D12 x y + D22 y y; its principal values and directions, false where a surface is singular.
struct Contact {
    double l1 = 0.0, l2 = 0.0;
    xyz v1{}, v2{};
};

bool contact(const Context& c, const X& x, Contact& k) {
    std::array<xyz, 6> ja{}, jb{};
    if (!c.a.surface.jet(x[0], x[1], 2, ja) || !c.b.surface.jet(x[2], x[3], 2, jb))
        return false;
    const xyz n = unit(ja[3].cross(ja[1])), e1 = unit(ja[3]), e2 = n.cross(e1);
    std::array<double, 3> D{};
    for (int s = 0; s < 2; ++s) {
        const std::array<xyz, 6>& j = s ? jb : ja; // S, Sv, Svv, Su, Suv, Suu
        const double e = j[3].dot(j[3]), f = j[3].dot(j[1]), g = j[1].dot(j[1]), det = e * g - f * f;
        if (!(det > 1e-30 * e * g))
            return false;
        std::array<std::array<double, 2>, 2> w{};
        for (int i = 0; i < 2; ++i) {
            const double p = j[3].dot(i ? e2 : e1), q = j[1].dot(i ? e2 : e1);
            w[i] = {(g * p - f * q) / det, (e * q - f * p) / det};
        }
        const auto form = [&](int i, int m) {
            return n.dot(j[5]) * w[i][0] * w[m][0] + n.dot(j[4]) * (w[i][0] * w[m][1] + w[i][1] * w[m][0]) + n.dot(j[2]) * w[i][1] * w[m][1];
        };
        D[0] += (s ? -1 : 1) * form(0, 0), D[1] += (s ? -1 : 1) * form(0, 1), D[2] += (s ? -1 : 1) * form(1, 1);
    }
    const double angle = 0.5 * std::atan2(2 * D[1], D[0] - D[2]), cs = std::cos(angle), sn = std::sin(angle);
    k.v1 = e1 * cs + e2 * sn, k.v2 = e2 * cs - e1 * sn;
    k.l1 = D[0] * cs * cs + 2 * D[1] * cs * sn + D[2] * sn * sn, k.l2 = D[0] * sn * sn - 2 * D[1] * cs * sn + D[2] * cs * cs;
    return true;
}

// Where the form is indefinite the section crosses itself along its two null directions; otherwise only its weak axis can carry
// a tangent curve, and no step shorter than `reach` tells one from the tolerance band of a point.
std::vector<xyz> branches(const Context& c, const X& x, double& reach) {
    Contact k;
    reach = 0.0;
    if (!contact(c, x, k))
        return {};
    const double weak = std::min(std::abs(k.l1), std::abs(k.l2)), strong = std::max(std::abs(k.l1), std::abs(k.l2));
    if (k.l1 * k.l2 < 0 && weak > 1e-6 * strong)
        return {unit(k.v1 * std::sqrt(std::abs(k.l2)) + k.v2 * std::sqrt(std::abs(k.l1))), unit(k.v1 * std::sqrt(std::abs(k.l2)) - k.v2 * std::sqrt(std::abs(k.l1)))};
    reach = weak > 0 ? std::sqrt(8 * c.tol / weak) : 0.0;
    return {std::abs(k.l1) < std::abs(k.l2) ? k.v1 : k.v2};
}

// A step from a tangent seed along dir that stays on both surfaces, tried at shrinking lengths not below reach.
bool leaves(const Context& c, const Node& seed, const xyz& dir, double reach, Node& out) {
    for (double h = 0.25 * c.h0; h >= 1e-3 * c.h0; h *= 0.25) {
        X y = predict(c, seed.x, dir * h);
        clamp(c, y);
        double gap = 0.0;
        if (correct(c, y, {-1, 0.0, seed.p, dir, h}, gap)) {
            out = {y, point(c, y)};
            return true;
        }
        if (0.25 * h < reach)
            break;
    }
    return false;
}

// GeomLib_CheckCurveOnSurface: the largest distance between the curve and the surface along its pcurve, sampled and refined.
double deviation(const session_cpp::Curve& c3d, const Curve2d& on, const Side& side) {
    const auto [t0, t1] = c3d.domain();
    const auto [p0, p1] = on.domain();
    const auto gap = [&](double t) { return c3d.point_at(t).distance(side.at(on.point_at(p0 + (t - t0) / (t1 - t0) * (p1 - p0)))); };
    // Under an analytic curve the pcurve may be the one with the pieces.
    const int n = std::clamp(16 * (c3d.kind() == session_cpp::Curve::Kind::Nurbs ? c3d.nurbs().cv_count() : std::max(c3d.nurbs().cv_count(), on.nurbs().cv_count())), 64, 2048);
    double worst = -1.0, at = t0;
    for (int i = 0; i <= n; ++i) {
        const double t = t0 + (t1 - t0) * i / n, d = gap(t);
        if (d > worst)
            worst = d, at = t;
    }
    double lo = std::max(t0, at - (t1 - t0) / n), hi = std::min(t1, at + (t1 - t0) / n);
    const double golden = 0.5 * (std::sqrt(5.0) - 1.0);
    for (int it = 0; it < 40; ++it) {
        const double m1 = hi - golden * (hi - lo), m2 = lo + golden * (hi - lo);
        if (gap(m1) > gap(m2))
            hi = m2;
        else
            lo = m1;
    }
    return std::max(worst, gap(0.5 * (lo + hi)));
}

// IntTools_FaceFace::ComputeTolReached3d with IntTools_Tools::ComputeTolerance: the curve's tolerance grows to the largest
// deviation from either surface along its pcurve, with OCCT's margin of 1e-5 for later trimming.
Curve make_curve(const Context& c, session_cpp::Curve c3d, Curve2d on_a, Curve2d on_b) {
    Curve curve{std::move(c3d), std::move(on_a), std::move(on_b), 0.0, false};
    for (const auto& [on, side] : {std::pair{&curve.pcurve_a, &c.a}, std::pair{&curve.pcurve_b, &c.b}})
        curve.tolerance = std::max(curve.tolerance, (1.0 + 1e-5) * deviation(curve.curve, *on, *side));
    curve.closed = curve.curve.point_at(curve.curve.domain().first).distance(curve.curve.point_at(curve.curve.domain().second)) <= std::max(curve.tolerance, 1e-7);
    return curve;
}

// gp_Pln of an affine planar patch: S(u, v) = o + u U + v V.
struct Plane3 {
    xyz o, u, v, n;
};

bool affine_plane(const Side& side, Plane3& plane) {
    const NurbsSurface& s = side.surface;
    if (side.implicit.kind != 1 || s.degree(0) != 1 || s.degree(1) != 1 || s.cv_count(0) != 2 || s.cv_count(1) != 2)
        return false;
    if (s.is_rational() && (s.weight(0, 0) != s.weight(1, 0) || s.weight(0, 0) != s.weight(0, 1) || s.weight(0, 0) != s.weight(1, 1)))
        return false;
    const auto [u0, u1] = s.domain(0);
    const auto [v0, v1] = s.domain(1);
    const xyz p00 = s.point(u0, v0), p10 = s.point(u1, v0), p01 = s.point(u0, v1), p11 = s.point(u1, v1);
    plane.u = (p10 - p00) / (u1 - u0), plane.v = (p01 - p00) / (v1 - v0);
    plane.o = p00 - plane.u * u0 - plane.v * v0;
    plane.n = unit(plane.u.cross(plane.v));
    return plane.n.length() > 0 && (p11 - p10 - p01 + p00).length() <= 1e-12 * ((p10 - p00).length() + (p01 - p00).length());
}

// ProjLib_Plane: the (u, v) of a point and of a direction in the plane.
std::array<double, 2> plane_uv(const Plane3& plane, const xyz& d) {
    const double e = plane.u.dot(plane.u), f = plane.u.dot(plane.v), g = plane.v.dot(plane.v), det = e * g - f * f;
    const double p = plane.u.dot(d), q = plane.v.dot(d);
    return {(g * p - f * q) / det, (e * q - f * p) / det};
}

// ClassifyLin2d: the parameters [p1, p2] of the line uv(s) = origin + s dir inside the face's uv box, false when it misses it
// or touches it over no more than tol; tol is a length, read along each parameter through the plane's scale.
bool classify_lin2d(const Side& side, const Plane3& plane, const std::array<double, 2>& origin, const std::array<double, 2>& dir,
                    double tol, double& p1, double& p2) {
    p1 = -infinity, p2 = infinity;
    for (int k = 0; k < 2; ++k) {
        const double lo = side.box[2 * k], hi = side.box[2 * k + 1], scale = (k ? plane.v : plane.u).length();
        if (std::abs(dir[k]) * scale <= 1e-12) {
            if (origin[k] < lo - tol / scale || origin[k] > hi + tol / scale)
                return false;
            continue;
        }
        const double a = (lo - origin[k]) / dir[k], b = (hi - origin[k]) / dir[k];
        p1 = std::max(p1, std::min(a, b)), p2 = std::min(p2, std::max(a, b));
    }
    return p2 - p1 > tol;
}

// PerformPlanes: IntAna_QuadQuadGeo on two planes (same within TolAng and TolTang, else parallel and apart, else a line), the
// line kept where it crosses both faces' uv boxes, its tolerance the larger face tolerance.
void perform_planes(const Side& a, const Plane3& pa, const Side& b, const Plane3& pb, double tol_f1, double tol_f2, Result& result) {
    const double tol_ang = 1e-8, tol_tang = tol_f1 + tol_f2;
    const xyz cross = pa.n.cross(pb.n);
    if (cross.length() <= tol_ang) {
        result.same_domain = std::abs((pb.o - pa.o).dot(pa.n)) <= tol_tang;
        return;
    }
    const double cosine = pa.n.dot(pb.n), h1 = pa.n.dot(pa.o), h2 = pb.n.dot(pb.o), det = 1.0 - cosine * cosine;
    const xyz dir = unit(cross), origin = pa.n * ((h1 - h2 * cosine) / det) + pb.n * ((h2 - h1 * cosine) / det);
    const std::array<double, 2> ua = plane_uv(pa, origin - pa.o), da = plane_uv(pa, dir), ub = plane_uv(pb, origin - pb.o), db = plane_uv(pb, dir);
    double p11 = 0, p12 = 0, p21 = 0, p22 = 0;
    if (!classify_lin2d(a, pa, ua, da, tol_tang, p11, p12) || !classify_lin2d(b, pb, ub, db, tol_tang, p21, p22) || p21 >= p12 || p22 <= p11)
        return;
    const double pmin = std::max(p11, p21), pmax = std::min(p12, p22);
    if (pmax - pmin <= tol_tang)
        return;
    // Lines as such where the planes are analytic: the parameter is the length along the section, on the pcurves too.
    const auto line = [&](const P3& p, const P3& q, bool exact) {
        if (exact)
            return session_cpp::Curve::line(p + (p - q) * (pmin / (pmax - pmin)), q - p, pmin, pmax);
        NurbsCurve curve = NurbsCurve::create(false, 1, {p, q});
        curve.set_domain(pmin, pmax);
        return session_cpp::Curve(curve);
    };
    Curve curve{line(P3(origin + dir * pmin), P3(origin + dir * pmax), a.analytic && b.analytic),
                line(P3(ua[0] + pmin * da[0], ua[1] + pmin * da[1], 0), P3(ua[0] + pmax * da[0], ua[1] + pmax * da[1], 0), a.analytic),
                line(P3(ub[0] + pmin * db[0], ub[1] + pmin * db[1], 0), P3(ub[0] + pmax * db[0], ub[1] + pmax * db[1], 0), b.analytic),
                std::max(tol_f1, tol_f2), false};
    // Its tangential tolerance (IntTools_Tools::ComputeIntRange): how far along the faces the section may lie where the planes meet at a shallow angle.
    const double angle = std::atan2(cross.length(), std::abs(cosine)), dt = angle > 0.5 * std::numbers::pi - 1e-12 ? tol_f2 : tol_f1 / std::tan(angle) + tol_f2 / std::sin(angle);
    curve.tolerance = std::max(curve.tolerance, std::hypot(dt, tol_f1));
    result.curves.push_back(std::move(curve));
    result.certified = true;
}

// The node halfway between two nodes of a line, corrected onto both surfaces.
Node midpoint(const Context& c, const Node& a, const Node& b) {
    X x;
    for (int k = 0; k < 4; ++k)
        x[k] = 0.5 * (a.x[k] + b.x[k]);
    const xyz chord = b.p - a.p;
    double gap = 0.0;
    correct(c, x, {-1, 0.0, a.p, unit(chord), 0.5 * chord.length()}, gap);
    return {x, point(c, x)};
}

bool interpolate(const Context& c, const std::vector<Node>& nodes, std::vector<double>& t, NurbsCurve& c3d, NurbsCurve& on_a, NurbsCurve& on_b) {
    std::vector<P3> p3, pa, pb;
    t = {0.0};
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (i)
            t.push_back(t.back() + (nodes[i].p - nodes[i - 1].p).length());
        p3.push_back(P3(nodes[i].p));
        pa.push_back(c.a.true_uv(nodes[i].x[0], nodes[i].x[1]));
        pb.push_back(c.b.true_uv(nodes[i].x[2], nodes[i].x[3]));
    }
    c3d = NurbsCurve::create_interpolated(p3, t), on_a = NurbsCurve::create_interpolated(pa, t), on_b = NurbsCurve::create_interpolated(pb, t);
    return c3d.is_valid() && on_a.is_valid() && on_b.is_valid();
}

// GeomInt_WLApprox in short: cubic interpolation of the walked points, one chord-length parameter for the curve and both pcurves,
// with points added where the curve leaves the surfaces by more than the target.
bool line_curve(const Context& c, const std::vector<Node>& nodes, Curve& out) {
    std::vector<Node> kept = {nodes.front()};
    for (size_t i = 1; i < nodes.size(); ++i)
        if ((nodes[i].p - kept.back().p).length() > 1e-9 * c.size || i + 1 == nodes.size())
            kept.push_back(nodes[i]);
    if (kept.size() < 2 || ((kept.back().p - kept.front().p).length() <= 1e-9 * c.size && kept.size() < 4))
        return false;
    if ((kept[kept.size() - 1].p - kept[kept.size() - 2].p).length() <= 1e-9 * c.size)
        kept.erase(kept.end() - 2);
    while (kept.size() < 5) {
        std::vector<Node> more = {kept.front()};
        for (size_t i = 1; i < kept.size(); ++i)
            more.push_back(midpoint(c, kept[i - 1], kept[i])), more.push_back(kept[i]);
        kept = std::move(more);
    }
    const double target = std::min(0.1 * c.tol, 1e-7 * std::max(1.0, c.size));
    NurbsCurve c3d, on_a, on_b;
    std::vector<double> t;
    for (int round = 0; round < 8; ++round) {
        if (!interpolate(c, kept, t, c3d, on_a, on_b))
            return false;
        std::vector<Node> more = {kept.front()};
        for (size_t i = 1; i < kept.size(); ++i) {
            const double m = 0.5 * (t[i - 1] + t[i]);
            const P3 p = c3d.point_at(m), ua = on_a.point_at(m), ub = on_b.point_at(m);
            const double apart = std::max(half_gap(c, kept[i - 1].x), half_gap(c, kept[i].x));
            if (kept.size() + more.size() < 8000 && std::max(p.distance(c.a.at(ua)), p.distance(c.b.at(ub))) > target + apart)
                more.push_back(midpoint(c, kept[i - 1], kept[i]));
            more.push_back(kept[i]);
        }
        if (more.size() == kept.size())
            break;
        kept = std::move(more);
        if (!interpolate(c, kept, t, c3d, on_a, on_b))
            return false;
    }
    out = make_curve(c, std::move(c3d), std::move(on_a), std::move(on_b));
    return true;
}

// IntPatch_Restriction: a line beside one bound of a face's box, where the surfaces are tangent and that bound itself lies on
// the other surface, is an edge of the face lying on the other, the edge/face interference's to find.
bool restriction(const Context& c, const std::vector<Node>& nodes) {
    for (int k = 0; k < 4; ++k) {
        const Side& side = k < 2 ? c.a : c.b;
        for (int end = 0; end < 2 && !side.closed[k % 2]; ++end) {
            const double b = bound(c, k, end);
            bool beside = true;
            for (size_t i = 0; i < nodes.size() && beside; i += std::max<size_t>(1, nodes.size() / 16)) {
                X y = nodes[i].x;
                y[k] = b;
                double sine = 0.0, gap = 0.0;
                tangent(c, nodes[i].x, sine);
                beside = sine < 1e-3 && correct(c, y, {k, b}, gap) && (point(c, y) - nodes[i].p).length() <= c.h0;
            }
            if (beside)
                return true;
        }
    }
    return false;
}

// Faces on one surface that share only a part of their boxes: a's samples over the box of the seeds lie on b, and the surfaces
// curve alike there, so that the box is no band of a tangent curve or point.
bool overlap(const Context& c, const std::vector<std::pair<Node, double>>& found) {
    if (found.size() < 3)
        return false;
    std::array<double, 4> box = {infinity, -infinity, infinity, -infinity};
    for (const auto& [n, sine] : found)
        for (int d = 0; d < 2; ++d)
            box[2 * d] = std::min(box[2 * d], n.x[d]), box[2 * d + 1] = std::max(box[2 * d + 1], n.x[d]);
    const double um = 0.5 * (box[0] + box[1]), vm = 0.5 * (box[2] + box[3]);
    const xyz middle = c.a.surface.point(um, vm);
    const double along = (c.a.surface.point(box[0], vm) - middle).length() + (c.a.surface.point(box[1], vm) - middle).length();
    const double across = (c.a.surface.point(um, box[2]) - middle).length() + (c.a.surface.point(um, box[3]) - middle).length();
    double strong = 0.0;
    for (size_t i : {size_t(0), found.size() / 2, found.size() - 1}) {
        Contact k;
        if (contact(c, found[i].first.x, k))
            strong = std::max({strong, std::abs(k.l1), std::abs(k.l2)});
    }
    return std::min(along, across) > 1000 * c.tol && strong * (along * along + across * across) < 2 * c.tol && covered(c.a, box, c.b, c.tol) >= 9;
}

// IntPatch_PrmPrmIntersection: walk every seed not yet on a line, both ways, across seams of closed faces; seeds where the surfaces only touch are points.
void march(const Context& c, const std::vector<std::pair<Node, double>>& found, Result& result) {
    std::vector<std::vector<Node>> lines;
    std::vector<Node> touching;
    struct Start {
        Node node;
        xyz dir;
        bool both;
    };
    std::vector<Start> queue;
    const auto wrap = [&](const Node& end, int k, const xyz& dir) {
        const Side& s = k < 2 ? c.a : c.b;
        if (k < 0 || !s.closed[k % 2])
            return;
        Node next = end;
        const double lo = bound(c, k, 0), hi = bound(c, k, 1);
        next.x[k] = std::abs(end.x[k] - lo) < std::abs(end.x[k] - hi) ? hi : lo;
        queue.push_back({next, dir, false});
    };
    for (const auto& [seed, sine] : found) {
        double s = 0.0, reach = 0.0;
        if (sine >= c.flat) {
            if (on_lines(c, lines, seed.p))
                continue;
            queue.push_back({seed, tangent(c, seed.x, s), true});
        } else {
            for (const xyz& dir : branches(c, seed.x, reach)) {
                Node ahead, behind;
                const bool forward = leaves(c, seed, dir, reach, ahead) && !on_lines(c, lines, ahead.p);
                const bool backward = leaves(c, seed, -dir, reach, behind) && !on_lines(c, lines, behind.p);
                if (forward || backward)
                    queue.push_back({seed, forward ? dir : -dir, forward && backward});
            }
            if (queue.empty() && !on_lines(c, lines, seed.p))
                touching.push_back(seed);
        }
        while (!queue.empty() && lines.size() < 512) {
            const Start start = queue.back();
            queue.pop_back();
            std::vector<Node> forward = {start.node}, backward = {start.node};
            int kf = -1, kb = -1;
            xyz lf = start.dir, lb = -start.dir;
            const Stop sf = walk(c, forward, start.dir, start.both ? &start.node : nullptr, kf, lf);
            Stop sb = Stop::Stuck;
            if (sf != Stop::Closed && start.both)
                sb = walk(c, backward, -start.dir, nullptr, kb, lb);
            std::vector<Node> nodes(backward.rbegin(), backward.rend());
            nodes.insert(nodes.end(), forward.begin() + 1, forward.end());
            // A short line between two seams has only its ends for nodes, and they are other lines' ends: its middle tells it apart.
            if (nodes.size() < 2 || (nodes.size() < 5 ? on_lines(c, lines, midpoint(c, nodes[(nodes.size() - 1) / 2], nodes[(nodes.size() - 1) / 2 + 1]).p, (nodes.back().p - nodes.front().p).length())
                : on_lines(c, lines, nodes[nodes.size() / 2].p) && on_lines(c, lines, nodes[nodes.size() / 4].p)))
                continue;
            lines.push_back(nodes);
            if (sf == Stop::Exit)
                wrap(forward.back(), kf, lf);
            if (sb == Stop::Exit)
                wrap(backward.back(), kb, lb);
        }
    }
    std::vector<std::vector<Node>> kept;
    for (const std::vector<Node>& line : lines) {
        double length = 0.0;
        for (size_t i = 1; i < line.size(); ++i)
            length += (line[i].p - line[i - 1].p).length();
        const bool edge = restriction(c, line);
        Curve curve;
        // A line shorter than a touch can be told from is one where it is known to pass.
        bool known = false;
        for (const xyz& p : c.through)
            for (const Node& n : line)
                known = known || (n.p - p).length() <= std::max(10 * c.tol, 1e-5 * c.size);
        if (length < std::max(10 * c.tol, 1e-5 * c.size) && !(known && length > 2 * c.tol)) {
            if (!edge)
                touching.push_back(line[line.size() / 2]);
            continue;
        }
        kept.push_back(line);
        if (!edge && line_curve(c, line, curve))
            result.curves.push_back(std::move(curve));
    }
    lines = std::move(kept);
    for (const Node& n : touching) {
        double sine = 0.0;
        tangent(c, n.x, sine);
        if (sine > 1e-2 || on_lines(c, lines, n.p))
            continue;
        const double gap = (c.a.surface.point(n.x[0], n.x[1]) - c.b.surface.point(n.x[2], n.x[3])).length();
        bool seen = false;
        for (Point& q : result.points)
            if ((xyz(q.point) - n.p).length() <= 1e-4 * c.size + c.tol) {
                seen = true;
                if (gap < q.tolerance)
                    q = {P3(n.p), c.a.true_uv(n.x[0], n.x[1]), c.b.true_uv(n.x[2], n.x[3]), (1.0 + 1e-5) * gap};
            }
        if (!seen)
            result.points.push_back({P3(n.p), c.a.true_uv(n.x[0], n.x[1]), c.b.true_uv(n.x[2], n.x[3]), (1.0 + 1e-5) * gap});
    }
}

// The (u, v) of a point of the section on a face's surface: Newton from the parameters of the point before it, which keeps the
// side of a seam, else from the nearest point of a grid of the surface and around its seams.
std::array<double, 2> locate(const Side& side, const P3& p, const std::array<double, 2>* near, double tol) {
    std::array<double, 2> uv{};
    for (int attempt = near ? 0 : 1; attempt < 2; ++attempt) {
        if (attempt == 0)
            uv = *near;
        else {
            const int nu = 8 * side.surface.span_count(0), nv = 8 * side.surface.span_count(1);
            double best = infinity;
            for (int i = 0; i <= nu; ++i)
                for (int k = 0; k <= nv; ++k) {
                    const double u = side.box[0] + i * (side.box[1] - side.box[0]) / nu, v = side.box[2] + k * (side.box[3] - side.box[2]) / nv;
                    const double d = (side.surface.point(u, v) - xyz(p)).length();
                    if (d < best)
                        best = d, uv = {u, v};
                }
        }
        for (int it = 0; it < 30; ++it) {
            const Jet j = jet(side.surface, uv[0], uv[1]);
            const xyz r = xyz(p) - j.p;
            const double e = j.du.dot(j.du), f = j.du.dot(j.dv), g = j.dv.dot(j.dv), det = e * g - f * f;
            if (!(det > 1e-30 * e * g))
                break;
            const double du = (g * j.du.dot(r) - f * j.dv.dot(r)) / det, dv = (e * j.dv.dot(r) - f * j.du.dot(r)) / det;
            uv = {uv[0] + du, uv[1] + dv};
            for (int d = 0; d < 2; ++d) {
                const double lo = side.box[2 * d], hi = side.box[2 * d + 1];
                uv[d] = attempt && side.closed[d] ? lo + std::fmod(std::fmod(uv[d] - lo, hi - lo) + (hi - lo), hi - lo) : std::clamp(uv[d], lo, hi);
            }
            if (std::abs(du) <= 1e-15 * (side.box[1] - side.box[0]) && std::abs(dv) <= 1e-15 * (side.box[3] - side.box[2]))
                break;
        }
        if ((side.surface.point(uv[0], uv[1]) - xyz(p)).length() <= tol)
            break;
    }
    return uv;
}

// The true parameters of a point of a section on a side: by closed form on an analytic surface, on the turn of the point before it or else in the box of the face.
P3 uv_at(const Side& side, const P3& p, std::array<double, 2>* near, double tol) {
    if (!side.analytic) {
        const std::array<double, 2> uv = locate(side, p, near, tol);
        if (near)
            *near = uv;
        return P3(uv[0], uv[1], 0);
    }
    std::pair<double, double> uv = side.form.closest_parameters(p);
    const std::vector<Vector> d = side.form.evaluate(uv.first, uv.second, 1);
    if (near && d[2].magnitude() <= 1e-9 * d[1].magnitude())
        uv.first = (*near)[0];
    uv = near ? side.form.adjusted(uv.first, uv.second, {(*near)[0], (*near)[0], (*near)[1], (*near)[1]}) : side.form.adjusted(uv.first, uv.second, side.box);
    if (near)
        *near = {uv.first, uv.second};
    return P3(uv.first, uv.second, 0);
}

// The piece [a, b] of a closed-form pcurve, moved by whole periods into the box of its face; false where a fitted pcurve does not reach.
bool pcurve_piece(const Curve2d& whole, const Side& side, double a, double b, Curve2d& out) {
    const auto [d0, d1] = whole.domain();
    const double e = 1e-12 * std::max(1.0, d1 - d0);
    out = whole;
    if (whole.kind() == session_cpp::Curve::Kind::Nurbs) {
        if (a < d0 - e || b > d1 + e)
            return false;
        if ((a > d0 + e || b < d1 - e) && !out.set_domain(std::max(a, d0), std::min(b, d1)))
            return false;
    } else if (!out.set_domain(a, b))
        return false;
    const P3 mid = out.point_at(0.5 * (out.domain().first + out.domain().second));
    const std::pair<double, double> in = side.form.adjusted(mid[0], mid[1], side.box);
    if (in.first != mid[0] || in.second != mid[1])
        out = out.shifted(in.first - mid[0], in.second - mid[1]);
    return true;
}

// The pcurves of an exact section on both faces in the curve's own parameter: as given by the closed form, else interpolated through the surface parameters of its
// points, with points added where a pcurve leaves the curve by more than the target.
bool exact_curve(const Context& c, const session_cpp::Curve& c3d, const std::array<const Curve2d*, 2>& given, double fit, Curve& out) {
    const std::vector<double> spans = c3d.nurbs().get_span_vector();
    std::vector<double> t = {spans.front()};
    for (size_t i = 0; i + 1 < spans.size(); ++i)
        for (int k = 1; k <= 4; ++k)
            if (spans[i] + 0.25 * k * (spans[i + 1] - spans[i]) > t.back() + 1e-9 * (spans.back() - spans.front()))
                t.push_back(spans[i] + 0.25 * k * (spans[i + 1] - spans[i]));
    t.back() = spans.back();
    const double target = std::min(0.1 * c.tol, 1e-9 * c.size) + 2.0 * fit;
    std::array<Curve2d, 2> on;
    for (int s = 0; s < 2; ++s)
        if (given[s])
            on[s] = *given[s];
    for (int round = 0; round < 12 && !(given[0] && given[1]); ++round) {
        const size_t n = t.size(), mid = n / 2;
        for (int s = 0; s < 2; ++s) {
            if (given[s])
                continue;
            const Side& side = s ? c.b : c.a;
            std::vector<P3> uv(n);
            const P3 first = uv_at(side, c3d.point_at(t[mid]), nullptr, c.tol);
            std::array<double, 2> last = {first[0], first[1]};
            const std::array<double, 2> middle = last;
            for (size_t i = mid; i < n; ++i)
                uv[i] = uv_at(side, c3d.point_at(t[i]), &last, c.tol);
            last = middle;
            for (size_t i = mid; i-- > 0;)
                uv[i] = uv_at(side, c3d.point_at(t[i]), &last, c.tol);
            const NurbsCurve fitted = NurbsCurve::create_interpolated(uv, t);
            if (!fitted.is_valid())
                return false;
            on[s] = fitted;
        }
        std::vector<double> more = {t.front()};
        for (size_t i = 1; i < n; ++i) {
            const double m = 0.5 * (t[i - 1] + t[i]);
            const P3 p = c3d.point_at(m);
            double apart = 0.0;
            for (int s = 0; s < 2; ++s)
                if (!given[s])
                    apart = std::max(apart, p.distance((s ? c.b : c.a).at(on[s].point_at(m))));
            if (n + more.size() < 4000 && apart > target)
                more.push_back(m);
            more.push_back(t[i]);
        }
        if (more.size() == n)
            break;
        t = std::move(more);
    }
    out = make_curve(c, c3d, std::move(on[0]), std::move(on[1]));
    return true;
}

// Whether a walked section is cut at the angle origin of a side's face where it passes it: a face that runs all the way around (IntPatch cuts a walking line at u = 0 and v = 0 of a periodic surface), and the quadric over whose angle the section is found, however far its face runs: both cylinders of a pair (IntPatch_ImpImpIntersection_6), the cylinder against a cone or sphere, the first of two cones, the cone against a sphere (IntAna_IntQuadQuad). Any other face that only straddles its origin has no bound there.
bool origin_cut(const Side& side, Surface::Kind other, int s, int d) {
    const double period = side.analytic ? side.form.period(d) : 0.0;
    if (!(period > 0) || std::floor((side.form.domain(d).second - 1e-9) / period) < std::ceil((side.form.domain(d).first + 1e-9) / period))
        return false;
    const Surface::Kind own = side.form.kind();
    return side.closed[d] || (d == 0 && ((own == Surface::Kind::Cylinder && (other == Surface::Kind::Cylinder || other == Surface::Kind::Cone || other == Surface::Kind::Sphere))
        || (own == Surface::Kind::Cone && ((other == Surface::Kind::Cone && s == 0) || other == Surface::Kind::Sphere))));
}

// IntPatch_ImpImpIntersection with GeomInt_LineConstructor: the closed-form sections of two recognised surfaces, each kept where it
// passes the uv boxes of both faces and cut where it crosses a seam of a face that closes around it or another section of the pair.
// False where no closed form applies.
bool closed_form(const Context& c, Result& result) {
    const std::array<const Side*, 2> sides = {&c.a, &c.b};
    std::array<Surface, 2> form;
    struct Seam {
        int side, d;
        double angle;
        bool walked; // of a face that passes the angle origin without ending there: only a section IntPatch walks is cut, and keeps a vertex
    };
    std::vector<Seam> seams;
    for (int s = 0; s < 2; ++s) {
        // Only a form the surface has to rounding: a loose one moves the section off the face it is to cut.
        form[s] = sides[s]->analytic ? sides[s]->form : Surface::recognized(sides[s]->surface, 1e-10 * sides[s]->size);
        if (form[s].kind() == Surface::Kind::Nurbs)
            return false;
    }
    // Where a face runs all or nearly all the way around, its section is cut at the angle its box starts from; where an analytic face passes its angle origin, a walked section is cut there and keeps a vertex (origin_cut).
    for (int s = 0; s < 2; ++s)
        for (int d = 0; d < 2; ++d) {
            const auto [lo, hi] = form[s].domain(d);
            const double period = form[s].period(d);
            if (period > 0 && hi - lo > 0.9 * period && !(sides[s]->analytic && !sides[s]->closed[d]))
                seams.push_back({s, d, lo, false});
            if (origin_cut(*sides[s], form[1 - s].kind(), s, d))
                seams.push_back({s, d, 0.0, true});
        }
    const intersection_analytic::Result exact = intersection_analytic::surface_surface(form[0], form[1], c.tol);
    if (!exact.done || exact.coincident)
        return false;
    // How far inside the uv box of a face a point of a section lies, negative outside, and beside which bound of it.
    const auto margin = [&](int s, const P3& p, std::array<int, 2>* bound = nullptr, std::array<double, 2>* each = nullptr) {
        const std::pair<double, double> uv = form[s].closest_parameters(p);
        double least = infinity;
        for (int d = 0; d < 2; ++d) {
            const auto [lo, hi] = form[s].domain(d);
            const double period = form[s].period(d), w = d ? uv.second : uv.first, e = 1e-9 * (hi - lo);
            const double inside = e + (period > 0 && w > hi ? -std::min(w - hi, lo + period - w) : std::min(w - lo, hi - w));
            least = std::min(least, inside);
            if (each)
                (*each)[d] = (inside - e) * form[s].point_at(uv.first + (d ? 0.0 : 1e-3 * (hi - lo)), uv.second + (d ? 1e-3 * (hi - lo) : 0.0)).distance(form[s].point_at(uv.first, uv.second)) / (1e-3 * (hi - lo));
            if (bound)
                (*bound)[d] = std::abs(std::remainder(w - lo, period > 0 ? period : 1e300)) <= e ? 0 : std::abs(std::remainder(w - hi, period > 0 ? period : 1e300)) <= e ? 1 : -1;
        }
        return least;
    };
    const auto both = [&](const intersection_analytic::Section& s, double t) { return std::min(margin(0, s.curve.point_at(t)), margin(1, s.curve.point_at(t))); };
    const auto bisect = [](double lo, double hi, const auto& low) {
        for (int it = 0; it < 60; ++it)
            (low(0.5 * (lo + hi)) ? lo : hi) = 0.5 * (lo + hi);
        return 0.5 * (lo + hi);
    };
    const int n = 256;
    std::vector<std::vector<double>> cuts(exact.curves.size()), kept(exact.curves.size());
    std::vector<bool> seamed(exact.curves.size(), false); // a closed section that starts on a seam
    for (size_t i = 0; i < exact.curves.size(); ++i) {
        const intersection_analytic::Section& s = exact.curves[i];
        const auto [t0, t1] = s.curve.domain();
        const auto at = [&](int k) { return t0 + (t1 - t0) * k / n; };
        // A section ends on the bound it crosses, to rounding: the bound it is furthest outside of where it has left the boxes. Where it grazes one bound, the rounding of a file putting it outside by less than a tenth of the tolerance, and crosses another beside it, that other ends it.
        const auto lengths = [&](double t) {
            std::array<std::array<double, 2>, 2> g{};
            margin(0, s.curve.point_at(t), nullptr, &g[0]);
            margin(1, s.curve.point_at(t), nullptr, &g[1]);
            return std::array<double, 4>{g[0][0], g[0][1], g[1][0], g[1][1]};
        };
        const auto least = [&](double t) { const std::array<double, 4> g = lengths(t); return std::min_element(g.begin(), g.end()) - g.begin(); };
        const auto leave = [&](double in, double out) {
            const double far = lengths(out)[least(out)] < -0.1 * c.tol ? bisect(in, out, [&](double t) { return lengths(t)[least(t)] >= -0.1 * c.tol; }) : bisect(in, out, [&](double t) { return both(s, t) >= 0; });
            const size_t k = least(far);
            cuts[i].push_back(lengths(in)[k] > 0 ? bisect(in, far, [&](double t) { return lengths(t)[k] >= 0; }) : far);
        };
        cuts[i] = {t0, t1};
        std::vector<double> m(n + 1);
        for (int k = 0; k <= n; ++k)
            m[k] = both(s, at(k));
        for (int k = 0; k < n; ++k) {
            if ((m[k] >= 0) != (m[k + 1] >= 0))
                leave(m[k] >= 0 ? at(k) : at(k + 1), m[k] >= 0 ? at(k + 1) : at(k));
            // A piece shorter than the sampling: the margin peaks between two samples outside.
            if (k > 0 && m[k] < 0 && m[k] >= m[k - 1] && m[k] >= m[k + 1]) {
                double lo = at(k - 1), hi = at(k + 1);
                const double golden = 0.5 * (std::sqrt(5.0) - 1.0);
                for (int it = 0; it < 60; ++it) {
                    const double m1 = hi - golden * (hi - lo), m2 = lo + golden * (hi - lo);
                    (both(s, m1) > both(s, m2) ? hi : lo) = both(s, m1) > both(s, m2) ? m2 : m1;
                }
                if (both(s, 0.5 * (lo + hi)) >= 0)
                    leave(0.5 * (lo + hi), at(k - 1)), leave(0.5 * (lo + hi), at(k + 1));
            }
            for (const Seam& seam : seams) {
                if (seam.walked && s.curve.kind() != session_cpp::Curve::Kind::Nurbs)
                    continue;
                const auto side_of = [&](double t) {
                    const std::pair<double, double> uv = form[seam.side].closest_parameters(s.curve.point_at(t));
                    return std::remainder((seam.d ? uv.second : uv.first) - seam.angle, form[seam.side].period(seam.d));
                };
                const double g0 = side_of(at(k)), g1 = side_of(at(k + 1));
                seamed[i] = seamed[i] || (k == 0 && std::abs(g0) <= 1e-9);
                // A sweep that starts or stops on the origin has its end where the walked line is cut.
                if (seam.walked && k == 0 && std::abs(g0) <= 1e-9)
                    kept[i].push_back(t0);
                if (seam.walked && k == n - 1 && std::abs(g1) <= 1e-9)
                    kept[i].push_back(t1);
                if ((g0 < 0) != (g1 < 0) && std::abs(g0) < 1.0 && std::abs(g1) < 1.0) {
                    cuts[i].push_back(bisect(at(k), at(k + 1), [&](double t) { return (side_of(t) < 0) == (g0 < 0); }));
                    if (seam.walked)
                        kept[i].push_back(cuts[i].back());
                }
            }
        }
        for (size_t j = 0; j < i; ++j)
            try {
                const intersection_analytic::Section& o = exact.curves[j];
                for (const curve_intersection::Hit& hit : curve_intersection::intersect(s.curve.nurbs(), o.curve.nurbs(), c.tol).points) {
                    cuts[i].push_back(session_cpp::Curve::wrapped(s.curve.closest_parameter(hit.point), t0, s.curve.period()));
                    cuts[j].push_back(session_cpp::Curve::wrapped(o.curve.closest_parameter(hit.point), o.curve.domain().first, o.curve.period()));
                }
            } catch (const std::exception&) {
            }
    }
    for (size_t i = 0; i < exact.curves.size(); ++i) {
        const intersection_analytic::Section& s = exact.curves[i];
        std::sort(cuts[i].begin(), cuts[i].end());
        // The ellipses of two cylinders keep a vertex where IntAna_QuadQuadGeo starts them: on the axis d1 - d2 of the one normal to d1 + d2 and the reverse, d the axes along their surfaces' lengths.
        const bool ellipses = s.curve.kind() == session_cpp::Curve::Kind::Ellipse && form[0].kind() == Surface::Kind::Cylinder && form[1].kind() == Surface::Kind::Cylinder;
        bool origin = ellipses;
        double start = 0.0;
        const auto [first, last] = s.curve.domain();
        const bool whole = !seamed[i] && std::all_of(cuts[i].begin(), cuts[i].end(), [&](double t) { return std::min(t - first, last - t) <= 1e-8 * (last - first); });
        // The whole circle or ellipse a plane cuts from a sphere, cylinder or cone starts where IntAna_QuadQuadGeo starts it, the surfaces as they say themselves.
        for (int flat = 0; flat < 2 && !origin && whole && s.curve.period() > 0 && s.curve.is_closed() && c.a.analytic && c.b.analytic; ++flat) {
            const Surface &plane = form[flat], &round = form[1 - flat];
            const int kind = round.kind() == Surface::Kind::Sphere ? 2 : round.kind() == Surface::Kind::Cylinder ? 3 : round.kind() == Surface::Kind::Cone ? 4 : 0;
            P3 at;
            if (plane.kind() != Surface::Kind::Plane || !bop::conic_origin(plane.origin(), plane.z(), kind, round.origin(), round.z(), round.radius(), s.curve.point_at(first), at))
                continue;
            start = session_cpp::Curve::wrapped(s.curve.closest_parameter(at), first, s.curve.period());
            if (s.curve.point_at(start).distance(at) > c.tol)
                continue;
            origin = true;
            cuts[i].push_back(start);
            std::sort(cuts[i].begin(), cuts[i].end());
        }
        // The closed quartic of two quadrics, which IntAna_IntQuadQuad sweeps, keeps the vertex it starts and ends at; a torus is walked, and so are two cylinders (IntPatch_ImpImpIntersection_6), which start at their seams.
        const auto quadric = [](const Surface& f) { return f.kind() == Surface::Kind::Sphere || f.kind() == Surface::Kind::Cylinder || f.kind() == Surface::Kind::Cone; };
        const bool swept = s.curve.kind() == session_cpp::Curve::Kind::Nurbs && (c.a.analytic || c.b.analytic) && quadric(form[0]) && quadric(form[1]) && !ellipses && (form[0].kind() != Surface::Kind::Cylinder || form[1].kind() != Surface::Kind::Cylinder)
            && s.curve.point_at(first).distance(s.curve.point_at(last)) <= c.tol;
        if (ellipses) {
            std::array<Vector, 2> axis;
            for (int k = 0; k < 2; ++k) {
                if (sides[k]->analytic) {
                    axis[k] = form[k].z();
                    continue;
                }
                const NurbsSurface& full = *sides[k]->full;
                const int d = full.degree(0) == 1 ? 0 : 1;
                const double m = 0.5 * (full.domain(1 - d).first + full.domain(1 - d).second);
                const P3 p0 = d ? full.point_at(m, full.domain(1).first) : full.point_at(full.domain(0).first, m), p1 = d ? full.point_at(m, full.domain(1).second) : full.point_at(full.domain(0).second, m);
                axis[k] = form[k].z() * (form[k].z().dot(p1 - p0) < 0 ? -1.0 : 1.0);
            }
            Vector sum = axis[0] + axis[1], difference = axis[0] - axis[1];
            sum.normalize_self();
            difference.normalize_self();
            const Vector x = std::abs(s.curve.z().dot(sum)) > std::abs(s.curve.z().dot(difference)) ? difference : sum;
            const P3 centre = s.curve.origin();
            start = session_cpp::Curve::wrapped(s.curve.closest_parameter(P3(centre[0] + x[0] * s.curve.radius(), centre[1] + x[1] * s.curve.radius(), centre[2] + x[2] * s.curve.radius())), s.curve.domain().first, s.curve.period());
            cuts[i].push_back(start);
            std::sort(cuts[i].begin(), cuts[i].end());
        }
        const auto at_start = [&](double t) { return (origin && std::abs(std::remainder(t - start, s.curve.period())) <= 1e-12) || (swept && (t == first || t == last)) || std::find(kept[i].begin(), kept[i].end(), t) != kept[i].end(); };
        // Any other closed section cut elsewhere runs on through its own origin.
        if (cuts[i].size() > 2 && s.curve.period() > 0 && s.curve.is_closed()) {
            cuts[i].erase(cuts[i].begin());
            cuts[i].back() = cuts[i].front() + s.curve.period();
        }
        for (size_t k = 0; k + 1 < cuts[i].size(); ++k) {
            const double a = cuts[i][k], b = cuts[i][k + 1];
            // A piece no wider than the slack of the boxes is the corner two cuts leave, no section.
            if (b - a <= 1e-8 * (s.curve.domain().second - s.curve.domain().first) || both(s, 0.5 * (a + b)) < 0 || s.curve.point_at(a).distance(s.curve.point_at(0.5 * (a + b))) + s.curve.point_at(b).distance(s.curve.point_at(0.5 * (a + b))) <= c.tol)
                continue;
            // IntPatch_Restriction: a tangent section along a bound of a face's box, no seam, is an edge of that face lying on the other.
            bool edge = false;
            for (int side = 0; side < 2 && s.tangent; ++side)
                for (int d = 0; d < 2; ++d) {
                    std::array<int, 2> first{}, bound{};
                    margin(side, s.curve.point_at(a), &first);
                    bool along = first[d] >= 0 && !(form[side].period(d) > 0 && form[side].domain(d).second - form[side].domain(d).first >= (1.0 - 1e-9) * form[side].period(d));
                    for (int m = 1; m <= 8 && along; ++m) {
                        margin(side, s.curve.point_at(a + (b - a) * m / 8), &bound);
                        along = bound[d] == first[d];
                    }
                    edge = edge || along;
                }
            if (edge)
                continue;
            session_cpp::Curve piece = s.curve;
            Curve curve;
            if (!piece.set_domain(a, b))
                return false;
            // Between freeform faces the section is the NURBS it always was; an analytic face has its pcurve as the closed form gives it.
            if (!c.a.analytic && !c.b.analytic)
                piece = session_cpp::Curve(piece.nurbs());
            std::array<Curve2d, 2> on;
            std::array<const Curve2d*, 2> given{};
            for (int k = 0; k < 2; ++k)
                if (sides[k]->analytic && pcurve_piece(k ? s.on_b : s.on_a, *sides[k], a, b, on[k]))
                    given[k] = &on[k];
            if (exact_curve(c, piece, given, s.fit, curve)) {
                // A piece its pcurves cannot follow is not of these patches: the form's extent is not the face's.
                if (curve.tolerance > std::max(c.tol, 1e-6 * c.size))
                    return false;
                curve.bound = {at_start(a), at_start(b)};
                result.curves.push_back(std::move(curve));
            } else
                return false;
        }
    }
    for (const intersection_analytic::Hit& hit : exact.points)
        if (margin(0, hit.point) >= 0 && margin(1, hit.point) >= 0)
            result.points.push_back({hit.point, uv_at(c.a, hit.point, nullptr, c.tol), uv_at(c.b, hit.point, nullptr, c.tol), c.tol});
    result.certified = true;
    return true;
}

// PostTreatFF in short: marched sections of one pair that cross each other away from their ends are cut there, so that a vertex stands at the crossing.
void cut_crossings(std::vector<Curve>& curves) {
    const int n = 64;
    std::vector<std::vector<double>> cuts(curves.size());
    std::vector<std::vector<xyz>> samples(curves.size());
    for (size_t i = 0; i < curves.size(); ++i)
        for (int k = 0; k <= n; ++k)
            samples[i].push_back(xyz(curves[i].curve.point_at(curves[i].curve.domain().first + (curves[i].curve.domain().second - curves[i].curve.domain().first) * k / n)));
    for (size_t i = 0; i < curves.size(); ++i)
        for (size_t j = 0; j < curves.size(); ++j) {
            if (i == j)
                continue;
            // Where the samples of one come nearest to the polyline of the other, the two curves are brought together.
            std::vector<double> d(n + 1, infinity);
            for (int k = 0; k <= n; ++k)
                for (int m = 0; m < n; ++m)
                    d[k] = std::min(d[k], segment_gap(samples[i][k], samples[j][m], samples[j][m + 1]));
            const NurbsCurve &a = curves[i].curve.nurbs(), &b = curves[j].curve.nurbs();
            const double step = (a.domain_end() - a.domain_start()) / n, tol = curves[i].tolerance + curves[j].tolerance + 1e-7;
            for (int k = 1; k < n; ++k) {
                // A crossing shows as a sharp dip of the distance; curves running side by side are no crossing.
                const double length = 0.5 * (samples[i][k + 1] - samples[i][k - 1]).length();
                if (d[k] > d[k - 1] || d[k] > d[k + 1] || d[k] > length || std::max(d[k - 1], d[k + 1]) < 0.2 * length)
                    continue;
                double t = a.domain_start() + k * step, s = 0.0, gap = infinity;
                for (int it = 0; it < 30 && gap > 0.1 * tol; ++it) {
                    s = b.closest_parameter(a.point_at(t));
                    t = std::clamp(a.closest_parameter(b.point_at(s)), t - step, t + step);
                    gap = a.point_at(t).distance(b.point_at(s));
                }
                Vector ta = a.tangent_at(t), tb = b.tangent_at(s);
                ta.normalize_self();
                tb.normalize_self();
                if (gap <= tol && ta.cross(tb).magnitude() > 0.1 && std::min(s - b.domain_start(), b.domain_end() - s) > 1e-3 * (b.domain_end() - b.domain_start()))
                    cuts[i].push_back(t);
            }
        }
    std::vector<Curve> result;
    for (size_t i = 0; i < curves.size(); ++i) {
        const double t0 = curves[i].curve.domain().first, t1 = curves[i].curve.domain().second, e = 1e-3 * (t1 - t0);
        std::vector<double> at = {t0};
        std::sort(cuts[i].begin(), cuts[i].end());
        for (double t : cuts[i])
            if (t > at.back() + e && t < t1 - e)
                at.push_back(t);
        at.push_back(t1);
        for (size_t k = 0; k + 1 < at.size() && at.size() > 2; ++k) {
            Curve piece = curves[i];
            if (!piece.curve.set_domain(at[k], at[k + 1]) || !piece.pcurve_a.set_domain(at[k], at[k + 1]) || !piece.pcurve_b.set_domain(at[k], at[k + 1])) {
                at.clear();
                break;
            }
            piece.closed = false;
            result.push_back(std::move(piece));
        }
        if (at.size() <= 2)
            result.push_back(curves[i]);
    }
    curves = std::move(result);
}

// A marched section that passes the angle origin of an analytic face is cut there and keeps a vertex where a walked one is (origin_cut), as closed_form does for its own.
void cut_origins(const Context& c, std::vector<Curve>& curves) {
    std::vector<Curve> result;
    for (const Curve& curve : curves) {
        const auto [t0, t1] = curve.curve.domain();
        std::vector<double> at = {t0};
        for (int s = 0; s < 2; ++s)
            for (int d = 0; d < 2; ++d) {
                const Side &side = s ? c.b : c.a, &other = s ? c.a : c.b;
                if (!origin_cut(side, other.analytic ? other.form.kind() : Surface::Kind::Nurbs, s, d))
                    continue;
                const double period = side.form.period(d);
                const auto off = [&](double t) { return std::remainder((s ? curve.pcurve_b : curve.pcurve_a).point_at(t)[d], period); };
                const int n = 64;
                for (int k = 0; k < n; ++k) {
                    double a = t0 + (t1 - t0) * k / n, b = t0 + (t1 - t0) * (k + 1) / n;
                    const double g0 = off(a), g1 = off(b);
                    // A section that runs along the origin crosses it only in its rounding.
                    if ((g0 < 0) == (g1 < 0) || std::abs(g0) > 1.0 || std::abs(g1) > 1.0 || std::abs(g1 - g0) <= 1e-9 * period)
                        continue;
                    for (int it = 0; it < 60; ++it)
                        ((off(0.5 * (a + b)) < 0) == (g0 < 0) ? a : b) = 0.5 * (a + b);
                    at.push_back(0.5 * (a + b));
                }
            }
        std::sort(at.begin(), at.end());
        at.erase(std::remove_if(at.begin() + 1, at.end(), [&](double t) { return std::min(t - t0, t1 - t) <= 1e-6 * (t1 - t0); }), at.end());
        at.push_back(t1);
        for (size_t k = 0; k + 1 < at.size() && at.size() > 2; ++k) {
            Curve piece = curve;
            if (!piece.curve.set_domain(at[k], at[k + 1]) || !piece.pcurve_a.set_domain(at[k], at[k + 1]) || !piece.pcurve_b.set_domain(at[k], at[k + 1])) {
                at.clear();
                break;
            }
            piece.closed = false;
            piece.bound = {k > 0 || curve.bound[0], k + 2 < at.size() || curve.bound[1]};
            result.push_back(std::move(piece));
        }
        if (at.size() <= 2)
            result.push_back(curve);
    }
    curves = std::move(result);
}

}

// IntTools_FaceFace::Perform: face tolerances grown by half the fuzzy value (at least Precision::Confusion), planes by
// PerformPlanes, other pairs over the faces' uv boxes: TangentFaces, else the section lines and tangent points.
Result intersect(const BRep& a, int face_a, const BRep& b, int face_b, double fuzzy, const std::vector<session_cpp::Point>& through) {
    Result result;
    try {
        const double fuzz = 0.5 * std::max(fuzzy, 1e-7);
        const double tol_f1 = std::max(a.m_faces[face_a].tolerance, 1e-7) + fuzz, tol_f2 = std::max(b.m_faces[face_b].tolerance, 1e-7) + fuzz;
        const double tol = tol_f1 + tol_f2;
        const Side sa = make_side(a, face_a, tol), sb = make_side(b, face_b, tol);
        Plane3 pa, pb;
        if (affine_plane(sa, pa) && affine_plane(sb, pb)) {
            perform_planes(sa, pa, sb, pb, tol_f1, tol_f2, result);
            return result;
        }
        if (same_domain(sa, sb, tol)) {
            result.same_domain = true;
            return result;
        }
        const double small = std::max(1e-300, std::min(sa.size, sb.size)), size = std::max(sa.size, sb.size);
        const Context c{sa, sb, tol, size, 1e-14 * std::max(1.0, size), 0.02 * small, 1e-9 * small, 0.1 * small, 1e-6, std::vector<xyz>(through.begin(), through.end())};
        std::vector<std::pair<Node, double>> found = seeds(c);
        for (const session_cpp::Point& p : through) {
            const std::pair<double, double> on_a = sa.surface.closest_parameters(p), on_b = sb.surface.closest_parameters(p);
            X x{on_a.first, on_a.second, on_b.first, on_b.second};
            double gap = 0.0, sine = 0.0;
            if (!correct(c, x, {}, gap, true))
                continue;
            tangent(c, x, sine);
            found.push_back({{x, point(c, x)}, sine});
        }
        // The closed form stands where every crossing the grids found lies on one of its sections.
        try {
            bool whole = closed_form(c, result);
            for (size_t i = 0; i < found.size() && whole; i += found.size() / 24 + 1) {
                bool on = found[i].second < c.flat;
                for (size_t k = 0; k < result.curves.size() && !on; ++k) {
                    const session_cpp::Curve& curve = result.curves[k].curve;
                    on = curve.point_at(bop::parameter(curve, P3(found[i].first.p), curve.domain().first, curve.domain().second)).distance(P3(found[i].first.p)) <= std::max(100 * c.tol, 1e-6 * c.size);
                }
                whole = on;
            }
            if (whole)
                return result;
        } catch (const std::exception&) {
        }
        result = {};
        if (overlap(c, found))
            result.same_domain = true;
        else {
            march(c, found, result);
            cut_crossings(result.curves);
            // A plane's sections IntPatch has in closed form: they are not walked.
            if (!(c.a.analytic && c.a.form.kind() == Surface::Kind::Plane) && !(c.b.analytic && c.b.form.kind() == Surface::Kind::Plane))
                cut_origins(c, result.curves);
        }
    } catch (const std::exception&) {
    }
    return result;
}

}
