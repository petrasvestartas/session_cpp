#include "nurbssurface_trimmed.h"
#include "closest.h"
#include "fmt/core.h"
#include "nurbssurface_trimmed.pb.h"
#include "primitives.h"
#include "remesh_nurbssurface_grid.h"
#include "tolerance.h"
#include <array>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace session_cpp {

namespace {

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Normal on the side of a C0 knot line that belongs to the triangle around center.
Vector crease_side_normal(
    const NurbsSurface& surface,
    const std::array<std::vector<double>, 2>& knots,
    const std::array<double, 2>& center,
    std::array<double, 2> uv
) {

    for (int dir = 0; dir < 2; ++dir)
        if (std::find(knots[dir].begin(), knots[dir].end(), uv[dir]) != knots[dir].end())
            uv[dir] = std::nextafter(uv[dir], center[dir]);

    return surface.normal_at(uv[0], uv[1]);
}

/// Winding-number test of (u, v) against a closed UV polygon.
bool point_in_polygon_2d(double u, double v, const std::vector<Point>& poly) {

    int winding = 0;
    const size_t n = poly.size();

    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const double x0 = poly[i][0];
        const double y0 = poly[i][1];
        const double x1 = poly[j][0];
        const double y1 = poly[j][1];
        const double cross = (x1 - x0) * (v - y0) - (y1 - y0) * (u - x0);

        if (y0 <= v && y1 > v && cross > 0.0)
            ++winding;

        if (y0 > v && y1 <= v && cross < 0.0)
            --winding;
    }

    return winding != 0;
}

/// True when (u, v) lies inside the outer loop and outside every hole.
bool inside_loops(double u, double v, const std::vector<std::vector<Point>>& loops_uv) {

    if (!point_in_polygon_2d(u, v, loops_uv[0]))
        return false;

    for (size_t li = 1; li < loops_uv.size(); ++li)
        if (point_in_polygon_2d(u, v, loops_uv[li]))
            return false;

    return true;
}

/// Surface point at (u, v) as a plain array.
std::array<double, 3> eval3(const NurbsSurface& srf, double u, double v) {

    const Point p = srf.point_at(u, v);

    return {p[0], p[1], p[2]};
}

/// Signed distance of the surface point at (u, v) to the plane (q, n).
double plane_field(
    const NurbsSurface& srf,
    const std::array<double, 3>& q,
    const std::array<double, 3>& n,
    double u,
    double v
) {

    const std::array<double, 3> p = eval3(srf, u, v);

    return (p[0] - q[0]) * n[0] + (p[1] - q[1]) * n[1] + (p[2] - q[2]) * n[2];
}

/// Newton steps of (u, v) onto the plane (q, n) along the field gradient.
void refine_crossing(
    const NurbsSurface& srf,
    const std::array<double, 3>& q,
    const std::array<double, 3>& n,
    double& u,
    double& v
) {

    for (int it = 0; it < 12; ++it) {
        const double fv = plane_field(srf, q, n, u, v);

        if (std::abs(fv) < 1e-9)
            break;

        const double h = 1e-4;
        std::array<double, 3> a = eval3(srf, u + h, v);
        std::array<double, 3> b = eval3(srf, u - h, v);
        std::array<double, 3> c = eval3(srf, u, v + h);
        std::array<double, 3> d = eval3(srf, u, v - h);
        const double gu = ((a[0] - b[0]) * n[0] + (a[1] - b[1]) * n[1] + (a[2] - b[2]) * n[2]) / (2 * h);
        const double gv = ((c[0] - d[0]) * n[0] + (c[1] - d[1]) * n[1] + (c[2] - d[2]) * n[2]) / (2 * h);
        const double g2 = gu * gu + gv * gv;

        if (g2 < 1e-20)
            break;

        u -= fv * gu / g2;
        v -= fv * gv / g2;
    }
}

/// Normal turn in degrees along dir over [t0, t1] on the line smid of the other direction, summed over four steps.
double span_turn(const NurbsSurface& srf, int dir, double t0, double t1, double smid) {

    double ma = 0.0;
    Vector pn(0.0, 0.0, 0.0);

    for (int k = 0; k <= 4; ++k) {
        const double t = t0 + k * (t1 - t0) / 4.0;
        const Vector nm = (dir == 0) ? srf.normal_at(t, smid) : srf.normal_at(smid, t);

        if (k > 0) {
            const double d = std::max(-1.0, std::min(1.0, pn.dot(nm)));
            ma += std::acos(d) * 180.0 / Tolerance::PI;
        }

        pn = nm;
    }

    return ma;
}

/// Largest distance of the quarter points along dir over [t0, t1] on the line smid from their chord.
double span_deviation(const NurbsSurface& srf, int dir, double t0, double t1, double smid) {

    const std::array<double, 3> p0 = (dir == 0) ? eval3(srf, t0, smid) : eval3(srf, smid, t0);
    const std::array<double, 3> p1 = (dir == 0) ? eval3(srf, t1, smid) : eval3(srf, smid, t1);
    double dev = 0.0;

    for (int k = 1; k <= 3; ++k) {
        const double fr = k / 4.0;
        const double tm = t0 + fr * (t1 - t0);
        const std::array<double, 3> pm = (dir == 0) ? eval3(srf, tm, smid) : eval3(srf, smid, tm);
        const double lx = p0[0] + fr * (p1[0] - p0[0]);
        const double ly = p0[1] + fr * (p1[1] - p0[1]);
        const double lz = p0[2] + fr * (p1[2] - p0[2]);
        const double dd =
            std::sqrt((pm[0] - lx) * (pm[0] - lx) + (pm[1] - ly) * (pm[1] - ly) + (pm[2] - lz) * (pm[2] - lz));

        if (dd > dev)
            dev = dd;
    }

    return dev;
}

/// Subdivisions per span along dir from the normal turn (max_angle_deg) and the chord deviation (chord_tol) at the mid line of the other direction.
std::vector<int> span_subdivisions(
    const NurbsSurface& srf,
    int dir,
    const std::vector<double>& sp,
    const std::vector<double>& osp,
    int deg,
    double max_angle_deg,
    double chord_tol
) {

    const int n = (int)sp.size() - 1;
    std::vector<int> subs(n, deg > 1 ? 2 : 1);
    const double smid = (osp.front() + osp.back()) * 0.5;

    for (int i = 0; i < n; ++i) {
        const double t0 = sp[i];
        const double t1 = sp[i + 1];

        if (deg > 1) {
            const double ma = span_turn(srf, dir, t0, t1, smid);
            subs[i] = std::max(subs[i], std::max(1, std::min((int)std::ceil(ma / max_angle_deg), 64)));
        }

        const double dev = span_deviation(srf, dir, t0, t1, smid);

        if (dev > chord_tol)
            subs[i] = std::max(subs[i], std::min((int)std::ceil(std::sqrt(dev / chord_tol)), 64));
    }

    return subs;
}

/// Grid parameters: each span of sp cut into subs[i] equal steps, ending on the last knot.
std::vector<double> span_parameters(const std::vector<double>& sp, const std::vector<int>& subs) {

    std::vector<double> out;

    for (int i = 0; i + 1 < (int)sp.size(); ++i)
        for (int s = 0; s < subs[i]; ++s)
            out.push_back(sp[i] + s * (sp[i + 1] - sp[i]) / subs[i]);

    out.push_back(sp.back());

    return out;
}

/// Span-adaptive grid parameters in u and v; false when the surface has no span in a direction.
bool span_grid(
    const NurbsSurface& srf,
    double max_angle_deg,
    double chord_tol,
    std::vector<double>& us,
    std::vector<double>& vs
) {

    const std::vector<double> usp = srf.get_span_vector(0);
    const std::vector<double> vsp = srf.get_span_vector(1);

    if (usp.size() < 2 || vsp.size() < 2)
        return false;

    us = span_parameters(usp, span_subdivisions(srf, 0, usp, vsp, srf.degree(0), max_angle_deg, chord_tol));
    vs = span_parameters(vsp, span_subdivisions(srf, 1, vsp, usp, srf.degree(1), max_angle_deg, chord_tol));

    return us.size() >= 2 && vs.size() >= 2;
}

/// Unit normal as a plain array, or none when degenerate.
bool unit3(const Vector& n, std::array<double, 3>& out) {

    const double nl = std::sqrt(n.magnitude_squared());

    if (nl < 1e-12)
        return false;

    out = {n[0] / nl, n[1] / nl, n[2] / nl};

    return true;
}

/// Parameters of the 2D segment crossing p1p2 x p3p4, or false when parallel or outside.
bool segment_intersection(
    const std::array<double, 2>& p1,
    const std::array<double, 2>& p2,
    const std::array<double, 2>& p3,
    const std::array<double, 2>& p4,
    double& s_out,
    double& t_out
) {

    const double d1u = p2[0] - p1[0];
    const double d1v = p2[1] - p1[1];
    const double d2u = p4[0] - p3[0];
    const double d2v = p4[1] - p3[1];
    const double den = d1u * d2v - d1v * d2u;

    if (std::abs(den) < 1e-20)
        return false;

    const double s = ((p3[0] - p1[0]) * d2v - (p3[1] - p1[1]) * d2u) / den;
    const double t = ((p3[0] - p1[0]) * d1v - (p3[1] - p1[1]) * d1u) / den;

    if (s < -1e-12 || s > 1.0 + 1e-12 || t < -1e-12 || t > 1.0 + 1e-12)
        return false;

    s_out = s;
    t_out = t;

    return true;
}

/// Newton refinement of a UV curve-curve crossing (ta, tb), clamped to the domains.
void newton_curve_curve(const NurbsCurve& ca, double& ta, const NurbsCurve& cb, double& tb, double tol) {

    for (int it = 0; it < 8; ++it) {
        const std::vector<Vector> da = ca.evaluate(ta, 1);
        const std::vector<Vector> db = cb.evaluate(tb, 1);
        const double fu = da[0][0] - db[0][0];
        const double fv = da[0][1] - db[0][1];

        if (std::hypot(fu, fv) < tol)
            break;

        const double j00 = da[1][0];
        const double j01 = -db[1][0];
        const double j10 = da[1][1];
        const double j11 = -db[1][1];
        const double den = j00 * j11 - j01 * j10;

        if (std::abs(den) < 1e-20)
            break;

        ta -= (fu * j11 - j01 * fv) / den;
        tb -= (j00 * fv - fu * j10) / den;
        const std::pair<double, double> adom = ca.domain();
        const std::pair<double, double> bdom = cb.domain();
        ta = std::min(std::max(ta, adom.first), adom.second);
        tb = std::min(std::max(tb, bdom.first), bdom.second);
    }
}

/// Signed area of a closed UV loop sampled at 64 parameters.
double loop_signed_area(const NurbsCurve& loop) {

    const int n = 64;
    const std::pair<double, double> ldom = loop.domain();
    const double l0 = ldom.first;
    const double l1 = ldom.second;
    double s = 0.0;
    Point prev = loop.point_at(l0);

    for (int i = 1; i <= n; ++i) {
        Point p = loop.point_at(l0 + (l1 - l0) * i / n);
        s += prev[0] * p[1] - p[0] * prev[1];
        prev = p;
    }

    return s * 0.5;
}

/// Plane coordinates of pt in the affine frame (p00, u_axis, v_axis).
Point project_to_uv(
    const Point& pt,
    const Point& p00,
    const Vector& u_axis,
    const Vector& v_axis,
    double u_len2,
    double v_len2
) {

    const Vector d = pt - p00;

    return Point(d.dot(u_axis) / u_len2, d.dot(v_axis) / v_len2, 0.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// VertexWelder
// ═══════════════════════════════════════════════════════════════════════════
/// Adds 3D points to a mesh, returning the existing vertex when one lies within tol.
class VertexWelder {
private:
    Mesh& mesh_; // Mesh receiving the vertices.
    double tol_; // Weld tolerance.
    double cell_; // Hash cell size.
    std::map<std::tuple<long long, long long, long long>, std::vector<std::pair<Point, size_t>>> cells_; // Vertices per cell.

public:

    /// Construct over a mesh with a weld tolerance and a hash cell size.
    VertexWelder(Mesh& mesh, double tol, double cell) : mesh_(mesh), tol_(tol), cell_(cell) {}

    /// Weld a 3D point, returning the existing vertex within tol or a new one.
    size_t weld(const Point& p) {

        const long long ci = (long long)std::floor(p[0] / cell_);
        const long long cj = (long long)std::floor(p[1] / cell_);
        const long long ck = (long long)std::floor(p[2] / cell_);

        for (long long di = -1; di <= 1; ++di)
            for (long long dj = -1; dj <= 1; ++dj)
                for (long long dk = -1; dk <= 1; ++dk) {
                    auto it = cells_.find(std::make_tuple(ci + di, cj + dj, ck + dk));

                    if (it == cells_.end())
                        continue;

                    for (const std::pair<Point, size_t>& entry : it->second)
                        if ((entry.first - p).magnitude_squared() <= tol_ * tol_)
                            return entry.second;
                }

        const size_t vk = mesh_.add_vertex(p);
        cells_[std::make_tuple(ci, cj, ck)].push_back({p, vk});

        return vk;
    }

    /// Weld the surface point at (u, v); a new vertex gets the surface normal.
    size_t weld_surface(const NurbsSurface& srf, double u, double v) {

        const size_t before = mesh_.number_of_vertices();
        const size_t vk = weld(srf.point_at(u, v));

        if (mesh_.number_of_vertices() > before) {
            const Vector nm = srf.normal_at(u, v);
            mesh_.vertex[vk].set_normal(nm[0], nm[1], nm[2]);
        }

        return vk;
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// Plane clipping
// ═══════════════════════════════════════════════════════════════════════════
/// Welded polygon of the part of a grid cell where the field is <= 0: kept corners and Newton-refined edge crossings in order.
std::vector<size_t> clip_cell(
    VertexWelder& welder,
    const NurbsSurface& srf,
    const std::array<double, 3>& q,
    const std::array<double, 3>& n,
    const std::array<double, 4>& cu,
    const std::array<double, 4>& cv,
    const std::array<double, 4>& fc
) {

    const bool in[4] = {fc[0] <= 0, fc[1] <= 0, fc[2] <= 0, fc[3] <= 0};
    std::vector<size_t> poly;

    for (int k = 0; k < 4; ++k) {
        const int kn = (k + 1) % 4;

        if (in[k])
            poly.push_back(welder.weld_surface(srf, cu[k], cv[k]));

        if (in[k] != in[kn]) {
            const double t = (std::abs(fc[k] - fc[kn]) > 1e-30) ? fc[k] / (fc[k] - fc[kn]) : 0.5;
            double u = cu[k] + (cu[kn] - cu[k]) * t;
            double v = cv[k] + (cv[kn] - cv[k]) * t;
            refine_crossing(srf, q, n, u, v);
            poly.push_back(welder.weld_surface(srf, u, v));
        }
    }

    return poly;
}

/// Fan a welded polygon into the mesh from its first vertex, skipping triangles with a repeated vertex.
void add_fan(Mesh& mesh, const std::vector<size_t>& poly) {

    for (size_t t = 1; t + 1 < poly.size(); ++t) {
        const size_t a = poly[0];
        const size_t b = poly[t];
        const size_t c = poly[t + 1];

        if (a == b || b == c || c == a)
            continue;

        mesh.add_face({a, b, c});
    }
}

/// Two UV triangles per cell of the grid us x vs.
std::vector<std::array<std::array<double, 2>, 3>> grid_triangles(
    const std::vector<double>& us,
    const std::vector<double>& vs
) {

    std::vector<std::array<std::array<double, 2>, 3>> tris;
    tris.reserve((us.size() - 1) * (vs.size() - 1) * 2);

    for (size_t i = 0; i + 1 < us.size(); ++i) {
        for (size_t j = 0; j + 1 < vs.size(); ++j) {
            const std::array<double, 2> a = {us[i], vs[j]};
            const std::array<double, 2> b = {us[i + 1], vs[j]};
            const std::array<double, 2> c = {us[i + 1], vs[j + 1]};
            const std::array<double, 2> d = {us[i], vs[j + 1]};
            tris.push_back({a, b, c});
            tris.push_back({a, c, d});
        }
    }

    return tris;
}

/// UV triangles clipped to the half (S-q).n <= 1e-9, each kept part fanned from its first corner.
std::vector<std::array<std::array<double, 2>, 3>> clip_triangles(
    const NurbsSurface& srf,
    const std::array<double, 3>& q,
    const std::array<double, 3>& n,
    const std::vector<std::array<std::array<double, 2>, 3>>& tris
) {

    const double eps = 1e-9;
    std::vector<std::array<std::array<double, 2>, 3>> next;

    for (const std::array<std::array<double, 2>, 3>& t : tris) {
        std::vector<std::array<double, 2>> poly;

        for (int e = 0; e < 3; ++e) {
            const std::array<double, 2>& p = t[e];
            const std::array<double, 2>& r = t[(e + 1) % 3];
            const double fp = plane_field(srf, q, n, p[0], p[1]);
            const double fr = plane_field(srf, q, n, r[0], r[1]);
            const bool pin = fp <= eps;
            const bool rin = fr <= eps;

            if (pin)
                poly.push_back(p);

            if (pin != rin) {
                const double tt = (std::abs(fp - fr) > 1e-30) ? fp / (fp - fr) : 0.5;
                double cu = p[0] + (r[0] - p[0]) * tt;
                double cv = p[1] + (r[1] - p[1]) * tt;
                refine_crossing(srf, q, n, cu, cv);
                poly.push_back({cu, cv});
            }
        }

        for (size_t w = 1; w + 1 < poly.size(); ++w)
            next.push_back({poly[0], poly[w], poly[w + 1]});
    }

    return next;
}

/// Mesh of UV triangles lifted onto the surface, seams welded within weld_tol, degenerate faces skipped.
Mesh weld_triangles(
    const NurbsSurface& srf,
    const std::vector<std::array<std::array<double, 2>, 3>>& tris,
    double weld_tol
) {

    Mesh result;
    VertexWelder welder(result, weld_tol, weld_tol);

    for (const std::array<std::array<double, 2>, 3>& t : tris) {
        const size_t a = welder.weld_surface(srf, t[0][0], t[0][1]);
        const size_t b = welder.weld_surface(srf, t[1][0], t[1][1]);
        const size_t c = welder.weld_surface(srf, t[2][0], t[2][1]);

        if (a == b || b == c || c == a)
            continue;

        result.add_face({a, b, c});
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// UVGraph
// ═══════════════════════════════════════════════════════════════════════════
/// Snapped UV vertices of the split graph: points within snap of each other share one id.
class UVVertexPool {
private:
    double snap_; // Snap distance.
    std::map<std::pair<long long, long long>, std::vector<int>> cells_; // Ids per cell.

public:
    std::vector<std::array<double, 2>> verts; // UV position per id.

    /// Construct with the snap distance.
    explicit UVVertexPool(double snap) : snap_(snap) {}

    /// Id of the vertex within snap of p, a new one when none.
    int id(const std::array<double, 2>& p) {

        const long long ci = (long long)std::floor(p[0] / snap_);
        const long long cj = (long long)std::floor(p[1] / snap_);

        for (int di = -1; di <= 1; ++di) {
            for (int dj = -1; dj <= 1; ++dj) {
                const auto bucket = cells_.find({ci + di, cj + dj});

                if (bucket == cells_.end())
                    continue;

                for (int vk : bucket->second) {
                    const std::array<double, 2>& q = verts[vk];

                    if (std::hypot(q[0] - p[0], q[1] - p[1]) <= snap_)
                        return vk;
                }
            }
        }

        const int vk = (int)verts.size();
        verts.push_back({p[0], p[1]});
        cells_[{ci, cj}].push_back(vk);

        return vk;
    }
};

/// Graph edge between two pool vertices on pcurve cidx (negative: a domain border) over [ta, tb].
struct SplitEdge {
    int a; // First pool vertex.
    int b; // Second pool vertex.
    int cidx; // Pcurve index, negative for a domain border.
    double ta; // Parameter at a.
    double tb; // Parameter at b.
};

/// Directed copy of a split edge: fwd when it runs a -> b.
struct HalfEdge {
    int tail; // Start vertex.
    int head; // End vertex.
    int eidx; // Split edge index.
    int fwd; // 1 when it runs a -> b.
};

/// UV domain of the split surface and the distance under which UV points snap together.
struct SplitDomain {
    double u0; // Start of the u domain.
    double u1; // End of the u domain.
    double v0; // Start of the v domain.
    double v1; // End of the v domain.
    double snap; // Snap distance in UV.
};

/// Sampled pcurve in UV with the parameter of each sample.
struct UVPoly {
    int cidx; // Pcurve index, negative for a domain border.
    std::vector<std::array<double, 2>> pts; // UV samples.
    std::vector<double> ts; // Parameter per sample.
};

/// Consecutive half-edges of a cycle on one pcurve.
struct Run {
    int cidx; // Pcurve index, negative for a domain border.
    int va; // First vertex.
    int vb; // Last vertex.
    double ta; // Parameter at va.
    double tb; // Parameter at vb.
};

/// Domain of the surface with the snap distance: tolerance carried from 3D into UV, else 1e-7 of the shorter side.
SplitDomain split_domain(const NurbsSurface& srf, double tolerance) {

    const std::pair<double, double> dom_u = srf.domain(0);
    const std::pair<double, double> dom_v = srf.domain(1);
    const double u0 = dom_u.first;
    const double u1 = dom_u.second;
    const double v0 = dom_v.first;
    const double v1 = dom_v.second;
    const double range_u = u1 - u0;
    const double range_v = v1 - v0;

    const std::vector<double> spans_u = srf.get_span_vector(0);
    const std::vector<double> spans_v = srf.get_span_vector(1);
    const int nu = std::max((int)spans_u.size() - 1, 1) * 4;
    const int nv = std::max((int)spans_v.size() - 1, 1) * 4;
    const double du = range_u / nu;
    const double dv = range_v / nv;
    const double mu = (u0 + u1) * 0.5;
    const double mv = (v0 + v1) * 0.5;
    const Point pmid = srf.point_at(mu, mv);
    const double uv_to_3d_u = pmid.distance(srf.point_at(std::min(mu + du, u1), mv)) / du;
    const double uv_to_3d_v = pmid.distance(srf.point_at(mu, std::min(mv + dv, v1))) / dv;
    double uv_to_3d = std::max(uv_to_3d_u, uv_to_3d_v);

    if (uv_to_3d < 1e-10)
        uv_to_3d = 1.0;

    if (tolerance > 0.0)
        return {u0, u1, v0, v1, std::max(1e-9, tolerance / uv_to_3d)};

    return {u0, u1, v0, v1, std::min(range_u, range_v) * 1e-7};
}

/// Snap a UV point onto the domain border when within the snap distance of it.
void snap_to_border(std::array<double, 2>& p, const SplitDomain& dom) {

    if (std::abs(p[0] - dom.u0) < dom.snap)
        p[0] = dom.u0;

    if (std::abs(p[0] - dom.u1) < dom.snap)
        p[0] = dom.u1;

    if (std::abs(p[1] - dom.v0) < dom.snap)
        p[1] = dom.v0;

    if (std::abs(p[1] - dom.v1) < dom.snap)
        p[1] = dom.v1;
}

/// One pass inserting the parameter midpoint of every chord farther than samp_tol from the curve; the count inserted.
int refine_samples(const NurbsCurve& crv, std::vector<std::array<double, 3>>& entries, double samp_tol) {

    int inserted = 0;
    size_t i = 0;

    while (i + 1 < entries.size()) {
        const std::array<double, 3> a = entries[i];
        const std::array<double, 3> b = entries[i + 1];
        const double tm = (a[0] + b[0]) * 0.5;
        const Point pm = crv.point_at(tm);
        const double exu = b[1] - a[1];
        const double exv = b[2] - a[2];
        const double l2 = exu * exu + exv * exv;
        double dev = 0.0;

        if (l2 > 1e-30) {
            const double s = ((pm[0] - a[1]) * exu + (pm[1] - a[2]) * exv) / l2;
            const double cx = a[1] + s * exu;
            const double cy = a[2] + s * exv;
            dev = std::hypot(pm[0] - cx, pm[1] - cy);
        }

        if (dev > samp_tol && entries.size() < 4096) {
            entries.insert(entries.begin() + i + 1, {tm, pm[0], pm[1]});
            inserted += 1;
            i += 2;
        } else {
            i += 1;
        }
    }

    return inserted;
}

/// Samples (t, u, v) of a pcurve: uniform in t, then up to six passes of chord refinement.
std::vector<std::array<double, 3>> sample_pcurve(const NurbsCurve& crv, double samp_tol) {

    const std::pair<double, double> cdom = crv.domain();
    const double ct0 = cdom.first;
    const double ct1 = cdom.second;
    const int n = std::min(std::max(crv.cv_count() * 4, 16), 2048);
    std::vector<std::array<double, 3>> entries;

    for (int i = 0; i <= n; ++i) {
        const double t = ct0 + (ct1 - ct0) * i / n;
        const Point p = crv.point_at(t);
        entries.push_back({t, p[0], p[1]});
    }

    for (int depth = 0; depth < 6; ++depth)
        if (refine_samples(crv, entries, samp_tol) == 0)
            break;

    return entries;
}

/// Polyline of pcurve cidx from its samples: clamped into the domain, snapped to the border, repeats dropped.
UVPoly clamp_samples(const std::vector<std::array<double, 3>>& entries, int cidx, const SplitDomain& dom) {

    UVPoly poly{cidx, {}, {}};

    for (const std::array<double, 3>& e : entries) {
        std::array<double, 2> p = {std::min(std::max(e[1], dom.u0), dom.u1), std::min(std::max(e[2], dom.v0), dom.v1)};
        snap_to_border(p, dom);

        if (!poly.pts.empty() && std::abs(p[0] - poly.pts.back()[0]) < 1e-15 &&
            std::abs(p[1] - poly.pts.back()[1]) < 1e-15)
            continue;

        poly.pts.push_back(p);
        poly.ts.push_back(e[0]);
    }

    return poly;
}

/// True when every point lies within the snap distance of one domain side.
bool on_border(const std::vector<std::array<double, 2>>& pts, const SplitDomain& dom) {

    bool on_u0 = true;
    bool on_u1 = true;
    bool on_v0 = true;
    bool on_v1 = true;

    for (const std::array<double, 2>& p : pts) {
        if (std::abs(p[0] - dom.u0) >= dom.snap)
            on_u0 = false;

        if (std::abs(p[0] - dom.u1) >= dom.snap)
            on_u1 = false;

        if (std::abs(p[1] - dom.v0) >= dom.snap)
            on_v0 = false;

        if (std::abs(p[1] - dom.v1) >= dom.snap)
            on_v1 = false;
    }

    return on_u0 || on_u1 || on_v0 || on_v1;
}

/// Length of a UV polyline.
double polyline_length(const std::vector<std::array<double, 2>>& pts) {

    double ext = 0.0;

    for (size_t k = 1; k < pts.size(); ++k)
        ext += std::hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1]);

    return ext;
}

/// Polylines of the valid pcurves that neither hug the border nor fall short of min_ext, then the four domain sides.
std::vector<UVPoly> uv_polylines(const std::vector<NurbsCurve>& pcurves, const SplitDomain& dom) {

    const double range_u = dom.u1 - dom.u0;
    const double range_v = dom.v1 - dom.v0;
    const double samp_tol = std::max(range_u, range_v) * 2e-5;
    const double min_ext = std::max(dom.snap * 8.0, std::min(range_u, range_v) * 1e-5);
    std::vector<UVPoly> polylines;

    for (int cidx = 0; cidx < (int)pcurves.size(); ++cidx) {
        if (!pcurves[cidx].is_valid())
            continue;

        const UVPoly poly = clamp_samples(sample_pcurve(pcurves[cidx], samp_tol), cidx, dom);

        if (poly.pts.size() >= 2 && !on_border(poly.pts, dom) && polyline_length(poly.pts) >= min_ext)
            polylines.push_back(poly);
    }

    polylines.push_back({-1, {{dom.u0, dom.v0}, {dom.u1, dom.v0}}, {dom.u0, dom.u1}});
    polylines.push_back({-2, {{dom.u1, dom.v0}, {dom.u1, dom.v1}}, {dom.v0, dom.v1}});
    polylines.push_back({-3, {{dom.u1, dom.v1}, {dom.u0, dom.v1}}, {dom.u1, dom.u0}});
    polylines.push_back({-4, {{dom.u0, dom.v1}, {dom.u0, dom.v0}}, {dom.v1, dom.v0}});

    return polylines;
}

/// UV bounds (umin, umax, vmin, vmax) of a polyline.
std::array<double, 4> uv_bounds(const std::vector<std::array<double, 2>>& pts) {

    std::array<double, 4> bounds = {pts[0][0], pts[0][0], pts[0][1], pts[0][1]};

    for (const std::array<double, 2>& p : pts) {
        bounds[0] = std::min(bounds[0], p[0]);
        bounds[1] = std::max(bounds[1], p[0]);
        bounds[2] = std::min(bounds[2], p[1]);
        bounds[3] = std::max(bounds[3], p[1]);
    }

    return bounds;
}

/// True when the bounds of B meet the bounds of A grown by snap.
bool boxes_overlap(const UVPoly& A, const UVPoly& B, double snap) {

    const std::array<double, 4> a = uv_bounds(A.pts);
    const std::array<double, 4> b = uv_bounds(B.pts);

    return !(b[0] > a[1] + snap || b[1] < a[0] - snap || b[2] > a[3] + snap || b[3] < a[2] - snap);
}

/// Parameter of a point along domain side cidx: u on the bottom and top sides, v on the left and right.
double border_parameter(int cidx, const std::array<double, 2>& hp) {
    return (cidx == -1 || cidx == -3) ? hp[0] : hp[1];
}

/// UV point of a crossing moved onto its pcurves, Newton-refined when both are pcurves, snapped to the border; ta and tb follow it.
std::array<double, 2> crossing_point(
    int acidx,
    double& ta,
    int bcidx,
    double& tb,
    const std::array<double, 2>& hit,
    const std::vector<NurbsCurve>& pcurves,
    const SplitDomain& dom
) {

    std::array<double, 2> hp = hit;

    if (acidx >= 0 && bcidx >= 0)
        newton_curve_curve(pcurves[acidx], ta, pcurves[bcidx], tb, dom.snap * 0.01);

    if (acidx >= 0) {
        const Point pa = pcurves[acidx].point_at(ta);
        hp = {pa[0], pa[1]};
    } else if (bcidx >= 0) {
        const Point pb = pcurves[bcidx].point_at(tb);
        hp = {pb[0], pb[1]};
    }

    snap_to_border(hp, dom);

    if (bcidx < 0)
        tb = border_parameter(bcidx, hp);

    if (acidx < 0)
        ta = border_parameter(acidx, hp);

    return hp;
}

/// Crossings of polylines pi and pj as events (fraction, u, v, parameter) on each crossed segment.
void add_crossings(
    const std::vector<UVPoly>& polylines,
    int pi,
    int pj,
    const std::vector<NurbsCurve>& pcurves,
    const SplitDomain& dom,
    std::map<std::pair<int, int>, std::vector<std::array<double, 4>>>& splits
) {

    const UVPoly& A = polylines[pi];
    const UVPoly& B = polylines[pj];

    for (int ia = 0; ia + 1 < (int)A.pts.size(); ++ia) {
        for (int ib = 0; ib + 1 < (int)B.pts.size(); ++ib) {
            double s;
            double t;

            if (!segment_intersection(A.pts[ia], A.pts[ia + 1], B.pts[ib], B.pts[ib + 1], s, t))
                continue;

            double ta = A.ts[ia] + (A.ts[ia + 1] - A.ts[ia]) * s;
            double tb = B.ts[ib] + (B.ts[ib + 1] - B.ts[ib]) * t;
            const std::array<double, 2> hit = {
                A.pts[ia][0] + (A.pts[ia + 1][0] - A.pts[ia][0]) * s,
                A.pts[ia][1] + (A.pts[ia + 1][1] - A.pts[ia][1]) * s
            };
            const std::array<double, 2> hp = crossing_point(A.cidx, ta, B.cidx, tb, hit, pcurves, dom);
            splits[{pi, ia}].push_back({s, hp[0], hp[1], ta});
            splits[{pj, ib}].push_back({t, hp[0], hp[1], tb});
        }
    }
}

/// Crossing events of every pair of overlapping polylines with at least one pcurve, keyed by (polyline, segment).
std::map<std::pair<int, int>, std::vector<std::array<double, 4>>> polyline_crossings(
    const std::vector<UVPoly>& polylines,
    const std::vector<NurbsCurve>& pcurves,
    const SplitDomain& dom
) {

    std::map<std::pair<int, int>, std::vector<std::array<double, 4>>> splits;

    for (int pi = 0; pi < (int)polylines.size(); ++pi)
        for (int pj = pi + 1; pj < (int)polylines.size(); ++pj)
            if ((polylines[pi].cidx >= 0 || polylines[pj].cidx >= 0) &&
                boxes_overlap(polylines[pi], polylines[pj], dom.snap))
                add_crossings(polylines, pi, pj, pcurves, dom, splits);

    return splits;
}

/// Graph edges along every polyline between consecutive pool vertices, its crossings inserted in order.
std::vector<SplitEdge> split_edges(
    const std::vector<UVPoly>& polylines,
    const std::map<std::pair<int, int>, std::vector<std::array<double, 4>>>& splits,
    UVVertexPool& pool
) {

    std::vector<SplitEdge> edges;

    for (int pi = 0; pi < (int)polylines.size(); ++pi) {
        const UVPoly& poly = polylines[pi];
        std::vector<std::pair<int, double>> chain;

        for (int i = 0; i < (int)poly.pts.size(); ++i) {
            chain.push_back({pool.id(poly.pts[i]), poly.ts[i]});
            const auto sit = splits.find({pi, i});

            if (i + 1 < (int)poly.pts.size() && sit != splits.end()) {
                std::vector<std::array<double, 4>> evs = sit->second;
                std::sort(evs.begin(), evs.end());

                for (const std::array<double, 4>& ev : evs)
                    chain.push_back({pool.id({ev[1], ev[2]}), ev[3]});
            }
        }

        for (int i = 0; i + 1 < (int)chain.size(); ++i) {
            const int a = chain[i].first;
            const int b = chain[i + 1].first;

            if (a == b)
                continue;

            edges.push_back({a, b, poly.cidx, chain[i].second, chain[i + 1].second});
        }
    }

    return edges;
}

/// Edges left after repeatedly dropping every edge with an end of degree one.
std::vector<SplitEdge> prune_dangling(const std::vector<SplitEdge>& edges) {

    std::vector<bool> alive(edges.size(), true);
    bool changed = true;

    for (size_t pass = 0; changed && pass <= edges.size(); ++pass) {
        changed = false;
        std::map<int, int> degree;

        for (size_t ei = 0; ei < edges.size(); ++ei) {
            if (!alive[ei])
                continue;

            degree[edges[ei].a] += 1;
            degree[edges[ei].b] += 1;
        }

        for (size_t ei = 0; ei < edges.size(); ++ei) {
            if (!alive[ei])
                continue;

            if (degree[edges[ei].a] == 1 || degree[edges[ei].b] == 1) {
                alive[ei] = false;
                changed = true;
            }
        }
    }

    std::vector<SplitEdge> live_edges;

    for (size_t ei = 0; ei < edges.size(); ++ei)
        if (alive[ei])
            live_edges.push_back(edges[ei]);

    return live_edges;
}

/// Two opposite half-edges per edge, the forward one at the even index.
std::vector<HalfEdge> half_edges(const std::vector<SplitEdge>& edges) {

    std::vector<HalfEdge> hes;

    for (int ei = 0; ei < (int)edges.size(); ++ei) {
        hes.push_back({edges[ei].a, edges[ei].b, ei, 1});
        hes.push_back({edges[ei].b, edges[ei].a, ei, 0});
    }

    return hes;
}

/// Successor of every half-edge around its face: the twin of an outgoing half-edge continues with its predecessor in the angle-sorted fan.
std::vector<int> next_half_edges(const std::vector<HalfEdge>& hes, const std::vector<std::array<double, 2>>& verts) {

    std::vector<std::vector<int>> out_map(verts.size());

    for (int hi = 0; hi < (int)hes.size(); ++hi)
        out_map[hes[hi].tail].push_back(hi);

    for (size_t vid = 0; vid < out_map.size(); ++vid) {
        std::vector<std::pair<double, int>> fan;

        for (int hi : out_map[vid]) {
            const double angle = std::atan2(verts[hes[hi].head][1] - verts[vid][1], verts[hes[hi].head][0] - verts[vid][0]);
            fan.push_back({angle, hi});
        }

        std::sort(fan.begin(), fan.end());

        for (size_t k = 0; k < fan.size(); ++k)
            out_map[vid][k] = fan[k].second;
    }

    std::vector<int> next_he(hes.size(), -1);

    for (const std::vector<int>& outs : out_map)
        for (size_t pos = 0; pos < outs.size(); ++pos)
            next_he[outs[pos] ^ 1] = outs[(pos + outs.size() - 1) % outs.size()];

    return next_he;
}

/// Cycles of at least two half-edges traced through next_he, each half-edge in one cycle.
std::vector<std::vector<int>> face_cycles(const std::vector<int>& next_he) {

    std::vector<bool> visited(next_he.size(), false);
    std::vector<std::vector<int>> faces;

    for (int hi = 0; hi < (int)next_he.size(); ++hi) {
        if (visited[hi])
            continue;

        std::vector<int> cycle;
        int cur = hi;

        while (cur >= 0 && !visited[cur]) {
            visited[cur] = true;
            cycle.push_back(cur);
            cur = next_he[cur];
        }

        if (cycle.size() >= 2)
            faces.push_back(cycle);
    }

    return faces;
}

/// Signed area of a half-edge cycle.
double cycle_area(
    const std::vector<int>& cycle,
    const std::vector<HalfEdge>& hes,
    const std::vector<std::array<double, 2>>& verts
) {

    double s = 0.0;

    for (int hi : cycle) {
        const std::array<double, 2>& a = verts[hes[hi].tail];
        const std::array<double, 2>& b = verts[hes[hi].head];
        s += a[0] * b[1] - b[0] * a[1];
    }

    return s * 0.5;
}

/// True when a cycle passes through a vertex of the domain border.
bool touches_border(const std::vector<int>& cycle, const std::vector<HalfEdge>& hes, const std::set<int>& border_vids) {

    for (int hi : cycle)
        if (border_vids.count(hes[hi].tail))
            return true;

    return false;
}

/// Face cycles by orientation: counter-clockwise faces with their area, clockwise holes clear of the domain border.
void classify_faces(
    const std::vector<std::vector<int>>& faces,
    const std::vector<HalfEdge>& hes,
    const std::vector<std::array<double, 2>>& verts,
    const std::vector<SplitEdge>& edges,
    double snap,
    std::vector<std::pair<std::vector<int>, double>>& pos_faces,
    std::vector<std::vector<int>>& neg_faces
) {

    std::set<int> border_vids;

    for (const SplitEdge& e : edges) {
        if (e.cidx < 0) {
            border_vids.insert(e.a);
            border_vids.insert(e.b);
        }
    }

    for (const std::vector<int>& cycle : faces) {
        const double area = cycle_area(cycle, hes, verts);

        if (area > snap * snap)
            pos_faces.push_back({cycle, area});
        else if (area < -snap * snap && !touches_border(cycle, hes, border_vids))
            neg_faces.push_back(cycle);
    }
}

/// Even-odd test of p against a half-edge cycle.
bool point_in_cycle(
    const std::array<double, 2>& p,
    const std::vector<int>& cycle,
    const std::vector<HalfEdge>& hes,
    const std::vector<std::array<double, 2>>& verts
) {

    bool inside = false;

    for (int hi : cycle) {
        const std::array<double, 2>& a = verts[hes[hi].tail];
        const std::array<double, 2>& b = verts[hes[hi].head];

        if ((a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0])
            inside = !inside;
    }

    return inside;
}

/// True when two cycles pass through the same set of vertices.
bool same_vertices(const std::vector<int>& a, const std::vector<int>& b, const std::vector<HalfEdge>& hes) {

    std::set<int> a_vids;
    std::set<int> b_vids;

    for (int hi : a)
        a_vids.insert(hes[hi].tail);

    for (int hi : b)
        b_vids.insert(hes[hi].tail);

    return a_vids == b_vids;
}

/// Holes per positive face: each hole goes to the smallest face that contains it and is not its own vertex ring.
std::vector<std::vector<std::vector<int>>> assign_holes(
    const std::vector<std::vector<int>>& neg_faces,
    const std::vector<std::pair<std::vector<int>, double>>& pos_faces,
    const std::vector<HalfEdge>& hes,
    const std::vector<std::array<double, 2>>& verts
) {

    std::vector<std::vector<std::vector<int>>> holes_of(pos_faces.size());

    for (const std::vector<int>& cycle : neg_faces) {
        const std::array<double, 2>& sample = verts[hes[cycle[0]].tail];
        int best = -1;
        double best_area = std::numeric_limits<double>::infinity();

        for (int fi = 0; fi < (int)pos_faces.size(); ++fi) {
            const std::vector<int>& fc = pos_faces[fi].first;
            const double area = pos_faces[fi].second;

            if (area < best_area && point_in_cycle(sample, fc, hes, verts) && !same_vertices(cycle, fc, hes)) {
                best = fi;
                best_area = area;
            }
        }

        if (best >= 0)
            holes_of[best].push_back(cycle);
    }

    return holes_of;
}

/// Runs of a cycle: consecutive half-edges on one pcurve merged.
std::vector<Run> cycle_runs(
    const std::vector<int>& cycle,
    const std::vector<HalfEdge>& hes,
    const std::vector<SplitEdge>& edges
) {

    std::vector<Run> runs;

    for (int hi : cycle) {
        const HalfEdge& he = hes[hi];
        const SplitEdge& e = edges[he.eidx];
        const double ta = he.fwd ? e.ta : e.tb;
        const double tb = he.fwd ? e.tb : e.ta;

        if (!runs.empty() && runs.back().cidx == e.cidx && runs.back().vb == he.tail) {
            runs.back().vb = he.head;
            runs.back().tb = tb;
        } else {
            runs.push_back({e.cidx, he.tail, he.head, ta, tb});
        }
    }

    return runs;
}

/// Pcurve piece of a run, trimmed to its parameters and oriented along it; false when the run cannot be cut.
bool run_piece(const Run& run, const std::vector<NurbsCurve>& pcurves, NurbsCurve& piece) {

    if (run.cidx < 0)
        return false;

    const NurbsCurve& crv = pcurves[run.cidx];
    const std::pair<double, double> cdom = crv.domain();
    const double c0 = cdom.first;
    const double c1 = cdom.second;
    const double lo = std::max(c0, std::min(run.ta, run.tb));
    const double hi_ = std::min(c1, std::max(run.ta, run.tb));
    piece = crv;

    if (hi_ - lo < (c1 - c0) - 1e-12 && hi_ - lo > 1e-14) {
        if (!piece.trim(lo, hi_))
            return false;
    } else if (hi_ - lo <= 1e-14 && !(run.va == run.vb && piece.is_closed())) {
        return false;
    }

    if (!piece.is_valid())
        return false;

    return run.ta <= run.tb || piece.reverse();
}

/// Pieces of a cycle: trimmed pcurve runs, straight UV segments where a run cannot be cut.
std::vector<NurbsCurve> cycle_to_segments(
    const std::vector<int>& cycle,
    const std::vector<HalfEdge>& hes,
    const std::vector<SplitEdge>& edges,
    const std::vector<std::array<double, 2>>& verts,
    const std::vector<NurbsCurve>& pcurves
) {

    std::vector<NurbsCurve> pieces;

    for (const Run& run : cycle_runs(cycle, hes, edges)) {
        NurbsCurve piece;

        if (run_piece(run, pcurves, piece)) {
            pieces.push_back(piece);
            continue;
        }

        const std::array<double, 2>& pa = verts[run.va];
        const std::array<double, 2>& pb = verts[run.vb];

        if (std::hypot(pb[0] - pa[0], pb[1] - pa[1]) > 1e-14)
            pieces.push_back(NurbsCurve::create(false, 1, {Point(pa[0], pa[1], 0.0), Point(pb[0], pb[1], 0.0)}));
    }

    return pieces;
}

/// Close a curve whose ends lie within tol by moving its last control point onto the first; true when it ends closed.
bool close_curve(NurbsCurve& curve, double tol) {

    if (curve.is_closed())
        return true;

    if (curve.point_at_start().distance(curve.point_at_end()) > tol)
        return false;

    const int last = curve.cv_count() - 1;
    double x;
    double y;
    double z;
    double w;
    double xe;
    double ye;
    double ze;
    double we;

    if (!curve.get_cv_4d(0, x, y, z, w) || !curve.get_cv_4d(last, xe, ye, ze, we))
        return false;

    return curve.set_cv_4d(last, x, y, z, we) && curve.is_closed();
}

/// Closed loop of a cycle: the joined pieces when they close, else the polygon through its vertices.
NurbsCurve cycle_to_loop(
    const std::vector<int>& cycle,
    const std::vector<HalfEdge>& hes,
    const std::vector<SplitEdge>& edges,
    const std::vector<std::array<double, 2>>& verts,
    const std::vector<NurbsCurve>& pcurves,
    double snap_uv
) {

    const std::vector<NurbsCurve> pieces = cycle_to_segments(cycle, hes, edges, verts, pcurves);

    if (pieces.empty())
        return NurbsCurve();

    const double join_tol = snap_uv * 4.0;
    std::vector<NurbsCurve> joined = NurbsCurve::join(pieces, join_tol);

    if (joined.size() == 1 && joined[0].is_valid() && close_curve(joined[0], join_tol))
        return joined[0];

    std::vector<Point> loop_pts;

    for (int hi : cycle) {
        const std::array<double, 2>& a = verts[hes[hi].tail];
        loop_pts.push_back(Point(a[0], a[1], 0.0));
    }

    loop_pts.push_back(Point(loop_pts[0][0], loop_pts[0][1], 0.0));

    return NurbsCurve::create(false, 1, loop_pts);
}

// ═══════════════════════════════════════════════════════════════════════════
// FlatMap64
// ═══════════════════════════════════════════════════════════════════════════
/// Open-addressing hash map from uint64 keys, linear probing, power-of-two capacity.
template <typename V> class FlatMap64 {
    static constexpr uint64_t EMPTY_KEY = ~uint64_t(0); // Key of a free slot.

    /// One hash slot.
    struct Slot {
        uint64_t key = EMPTY_KEY; // Key, EMPTY_KEY when free.
        V value; // Stored value.
    };

    std::vector<Slot> slots_; // Slot table.
    size_t size_ = 0; // Occupied slots.
    size_t shift_ = 64; // Right shift mapping a hash into the table.

    /// Smallest r with 2^r >= n.
    static size_t log2_pot(size_t n) {

        size_t r = 0;

        while ((size_t(1) << r) < n)
            ++r;

        return r;
    }

    /// Home slot of a key.
    size_t probe(uint64_t key) const {
        return (size_t)((key * uint64_t(0x9E3779B97F4A7C15ULL)) >> shift_);
    }

    /// Double the table and reinsert every entry.
    void grow() {

        const size_t new_cap = slots_.empty() ? 16 : slots_.size() * 2;
        std::vector<Slot> old = std::move(slots_);
        slots_.assign(new_cap, Slot{EMPTY_KEY, V{}});
        shift_ = 64 - log2_pot(new_cap);
        size_ = 0;

        for (Slot& s : old)
            if (s.key != EMPTY_KEY)
                insert_impl(s.key, std::move(s.value));
    }

    /// Insert or overwrite without growing.
    void insert_impl(uint64_t key, V val) {

        size_t mask = slots_.size() - 1;
        size_t i = probe(key);

        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) {
                slots_[i].value = std::move(val);

                return;
            }

            i = (i + 1) & mask;
        }

        slots_[i].key = key;
        slots_[i].value = std::move(val);
        ++size_;
    }

public:

    /// Construct an empty map.
    FlatMap64() = default;

    /// Reserve room for n entries at half load.
    void reserve(size_t n) {

        size_t need = n * 2;

        if (need <= slots_.size())
            return;

        size_t cap = 16;

        while (cap < need)
            cap *= 2;

        std::vector<Slot> old = std::move(slots_);
        slots_.assign(cap, Slot{EMPTY_KEY, V{}});
        shift_ = 64 - log2_pot(cap);
        size_ = 0;

        for (Slot& s : old)
            if (s.key != EMPTY_KEY)
                insert_impl(s.key, std::move(s.value));
    }

    /// Value of a key, null when absent.
    V* find(uint64_t key) {

        if (slots_.empty())
            return nullptr;

        size_t mask = slots_.size() - 1;
        size_t i = probe(key);

        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key)
                return &slots_[i].value;

            i = (i + 1) & mask;
        }

        return nullptr;
    }

    /// Const value of a key, null when absent.
    const V* find(uint64_t key) const {

        if (slots_.empty())
            return nullptr;

        size_t mask = slots_.size() - 1;
        size_t i = probe(key);

        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key)
                return &slots_[i].value;

            i = (i + 1) & mask;
        }

        return nullptr;
    }

    /// Value of a key, inserted default-constructed when absent.
    V& operator[](uint64_t key) {

        if (size_ * 2 >= slots_.size())
            grow();

        size_t mask = slots_.size() - 1;
        size_t i = probe(key);

        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key)
                return slots_[i].value;

            i = (i + 1) & mask;
        }

        slots_[i].key = key;
        slots_[i].value = V{};
        ++size_;

        return slots_[i].value;
    }

    /// Remove a key, backward-shifting the probe run.
    void erase(uint64_t key) {

        if (slots_.empty())
            return;

        size_t mask = slots_.size() - 1;
        size_t i = probe(key);

        while (slots_[i].key != EMPTY_KEY) {
            if (slots_[i].key == key) {
                --size_;
                size_t j = i;

                for (size_t step = 0; step < slots_.size(); ++step) {
                    j = (j + 1) & mask;

                    if (slots_[j].key == EMPTY_KEY)
                        break;

                    size_t k = probe(slots_[j].key);
                    bool move = (i <= j) ? (k <= i || k > j) : (k <= i && k > j);

                    if (move) {
                        slots_[i] = std::move(slots_[j]);
                        i = j;
                    }
                }

                slots_[i].key = EMPTY_KEY;

                return;
            }

            i = (i + 1) & mask;
        }
    }

    /// Free every slot.
    void clear() {

        for (Slot& s : slots_)
            s.key = EMPTY_KEY;

        size_ = 0;
    }

    /// Number of entries.
    size_t size() const {
        return size_;
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// Delaunay2D
// ═══════════════════════════════════════════════════════════════════════════
/// UV vertex of the triangulation.
struct Vertex2D {
    double x = 0.0; // U coordinate.
    double y = 0.0; // V coordinate.
};

/// Triangle with per-edge neighbours; edge k is opposite vertex k.
struct Triangle {
    int v[3] = {-1, -1, -1}; // Vertex indices.
    int adj[3] = {-1, -1, -1}; // Neighbour across each edge, -1 on the hull.
    bool constrained[3] = {false, false, false}; // True where an edge is a constraint.
    bool alive = true; // False once removed.
};

/// Incremental constrained Delaunay triangulation in UV with Bowyer-Watson insertion.
class Delaunay2D {
private:

    /// Edge of the cavity polygon.
    struct BEdge {
        int e0; // Start vertex.
        int e1; // End vertex.
        bool constrained = false; // True when the edge is a constraint.
    };

    int visit_epoch_ = 0; // Stamp of the current search.
    std::vector<int> visit_stamp_; // Last search stamp per triangle.
    mutable std::vector<int> bad_; // Triangles whose circumcircle holds the new point.
    mutable std::vector<BEdge> polygon_; // Cavity boundary of the current insertion.

public:
    std::vector<Vertex2D> vertices; // Vertices, the super triangle first.
    std::vector<Triangle> triangles; // Triangle pool, dead ones flagged.
    int super_v[3] = {-1, -1, -1}; // Super triangle vertices.
    FlatMap64<std::pair<int, int>> edge_map; // Hull edge -> (triangle, edge index).
    int last_found = 0; // Triangle the last locate ended in.

    /// Order-independent key of an edge.
    static uint64_t edge_key(int a, int b) {
        return ((uint64_t)(uint32_t)std::min(a, b) << 32) | (uint64_t)(uint32_t)std::max(a, b);
    }

    /// Construct with a super triangle around the box.
    Delaunay2D(double xmin, double ymin, double xmax, double ymax);

    /// Insert a point and return its vertex index, the existing one when coincident.
    int insert(double x, double y);

    /// Force the edge v0-v1 into the triangulation by flipping the edges it crosses.
    void insert_constraint(int v0, int v1);

    /// Drop the triangles touching the super triangle.
    void cleanup();

    /// Vertex index triples of the live triangles.
    std::vector<std::array<int, 3>> get_triangles() const;

    /// Positive when d lies inside the circumcircle of a, b, c.
    static double in_circumcircle(
        double ax,
        double ay,
        double bx,
        double by,
        double cx,
        double cy,
        double dx,
        double dy
    );

    /// Twice the signed area of a, b, c.
    static double orient2d(double ax, double ay, double bx, double by, double cx, double cy);

    /// Triangle containing (x, y) by walking from start_tri, -1 when none.
    int locate(double x, double y, int start_tri = 0) const;

private:

    /// True when triangle ti has vertex v.
    bool has_vertex(int ti, int v) const {
        return triangles[ti].v[0] == v || triangles[ti].v[1] == v || triangles[ti].v[2] == v;
    }

    /// Corner of triangle ti holding vertex v, -1 when none does.
    int vertex_index(int ti, int v) const;

    /// Vertex of triangle ti across its edge shared with triangle nb, -1 when they are not neighbours.
    int opposite_vertex(int ti, int nb) const;

    /// Vertex of triangle start within 1e-6 of (x, y), -1 when none.
    int find_coincident(int start, double x, double y) const;

    /// True when (x, y) lies inside the circumcircle of triangle ti.
    bool circumcircle_contains(int ti, double x, double y) const;

    /// Fill bad_ with the triangles whose circumcircle holds (x, y), grown from start across unconstrained edges.
    void collect_cavity(int start, double x, double y);

    /// Fill polygon_ with the edges of the bad_ triangles that face a good neighbour or the hull.
    void cavity_polygon();

    /// Replace the bad_ triangles by a fan from vertex vi to the polygon_ edges.
    void fill_cavity(int vi);

    /// Mark v0-v1 constrained when it already is a triangle edge; false when it is not.
    bool constrain_existing(int v0, int v1);

    /// Lowest live triangle with vertex v, -1 when none.
    int first_triangle_at(int v) const;

    /// Triangle around v0 whose opposite edge the segment v0-v1 crosses, with that edge's left and right ends; -1 when none.
    int first_crossed(int start_ti, int v0, int v1, int& ivl, int& ivr) const;

    /// Walk the triangles crossed by v0-v1 from intersected[0], collecting the vertices left and right of it.
    void walk_crossed(
        int v0,
        int v1,
        int ivl,
        int ivr,
        std::vector<int>& poly_l,
        std::vector<int>& poly_r,
        std::vector<int>& intersected
    ) const;

    /// Replace the crossed triangles by the two fans on either side of v0-v1 and constrain it.
    void retriangulate(
        int v0,
        int v1,
        const std::vector<int>& poly_l,
        const std::vector<int>& poly_r,
        const std::vector<int>& intersected
    );

    /// Constrain every edge of a triangle from first_new on that its older neighbour holds constrained.
    void inherit_constraints(int first_new);

    /// Mark the edge v0-v1 constrained in every live triangle that has it.
    void mark_edge(int v0, int v1);

    /// New counter-clockwise triangle over three vertices; skipped when degenerate.
    void add_triangle(int pa, int pb, int pc);

    /// Record the hull edges of triangle ti in the edge map.
    void register_edges(int ti);

    /// Drop the hull edges of triangle ti from the edge map and its neighbours.
    void unregister_edges(int ti);
};

double Delaunay2D::in_circumcircle(
    double ax,
    double ay,
    double bx,
    double by,
    double cx,
    double cy,
    double dx,
    double dy
) {

    const double adx = ax - dx;
    const double ady = ay - dy;
    const double bdx = bx - dx;
    const double bdy = by - dy;
    const double cdx = cx - dx;
    const double cdy = cy - dy;

    return (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy) + (bdx * bdx + bdy * bdy) * (cdx * ady - adx * cdy) +
        (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);
}

double Delaunay2D::orient2d(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

void Delaunay2D::register_edges(int ti) {

    const Triangle& t = triangles[ti];

    for (int k = 0; k < 3; ++k) {
        const int a = t.v[(k + 1) % 3];
        const int b = t.v[(k + 2) % 3];
        const uint64_t key = edge_key(a, b);
        std::pair<int, int>* val = edge_map.find(key);

        if (val) {
            const int oti = val->first;
            const int ok = val->second;
            triangles[ti].adj[k] = oti;
            triangles[oti].adj[ok] = ti;
            edge_map.erase(key);
        } else {
            edge_map[key] = {ti, k};
        }
    }
}

void Delaunay2D::unregister_edges(int ti) {

    const Triangle& t = triangles[ti];

    for (int k = 0; k < 3; ++k) {
        const int a = t.v[(k + 1) % 3];
        const int b = t.v[(k + 2) % 3];
        const uint64_t key = edge_key(a, b);
        const int adj_ti = t.adj[k];

        if (adj_ti >= 0 && triangles[adj_ti].alive) {
            Triangle& adj = triangles[adj_ti];

            for (int kk = 0; kk < 3; ++kk)
                if (adj.adj[kk] == ti) {
                    adj.adj[kk] = -1;
                    edge_map[edge_key(adj.v[(kk + 1) % 3], adj.v[(kk + 2) % 3])] = {adj_ti, kk};
                    break;
                }
        } else {
            std::pair<int, int>* val = edge_map.find(key);

            if (val && val->first == ti)
                edge_map.erase(key);
        }
    }
}

Delaunay2D::Delaunay2D(double xmin, double ymin, double xmax, double ymax) {

    const double dx = xmax - xmin;
    const double dy = ymax - ymin;
    const double d = std::max(dx, dy);
    const double cx = (xmin + xmax) * 0.5;
    const double cy = (ymin + ymax) * 0.5;
    const double scale = 20.0;
    vertices.push_back({cx - scale * d, cy - scale * d});
    vertices.push_back({cx + scale * d, cy - scale * d});
    vertices.push_back({cx, cy + scale * d});
    super_v[0] = 0;
    super_v[1] = 1;
    super_v[2] = 2;
    triangles.reserve(4096);
    vertices.reserve(2048);
    edge_map.reserve(2048);
    visit_stamp_.reserve(4096);
    bad_.reserve(32);
    polygon_.reserve(32);
    Triangle t;
    t.v[0] = 0;
    t.v[1] = 1;
    t.v[2] = 2;
    triangles.push_back(t);
    register_edges(0);
}

int Delaunay2D::locate(double x, double y, int start_tri) const {

    if (start_tri < 0 || start_tri >= (int)triangles.size() || !triangles[start_tri].alive) {
        start_tri = (int)triangles.size() - 1;

        while (start_tri >= 0 && !triangles[start_tri].alive)
            --start_tri;

        if (start_tri < 0)
            return -1;
    }

    int cur = start_tri;
    const int max_iter = (int)triangles.size();

    for (int iter = 0; iter < max_iter; ++iter) {
        const Triangle& tri = triangles[cur];
        bool moved = false;

        for (int k = 0; k < 3; ++k) {
            const int a = tri.v[k];
            const int b = tri.v[(k + 1) % 3];

            if (orient2d(vertices[a].x, vertices[a].y, vertices[b].x, vertices[b].y, x, y) < 0) {
                const int opp = (k + 2) % 3;

                if (tri.adj[opp] >= 0 && triangles[tri.adj[opp]].alive) {
                    cur = tri.adj[opp];
                    moved = true;
                    break;
                }
            }
        }

        if (!moved)
            return cur;
    }

    return cur;
}

int Delaunay2D::insert(double x, double y) {

    const int start = locate(x, y, last_found);
    const int existing = find_coincident(start, x, y);

    if (existing >= 0)
        return existing;

    const int vi = (int)vertices.size();
    vertices.push_back({x, y});
    collect_cavity(start, x, y);

    if (bad_.empty()) {
        vertices.pop_back();

        return -1;
    }

    cavity_polygon();
    fill_cavity(vi);
    last_found = (int)triangles.size() - 1;

    return vi;
}

int Delaunay2D::vertex_index(int ti, int v) const {

    for (int k = 0; k < 3; ++k)
        if (triangles[ti].v[k] == v)
            return k;

    return -1;
}

int Delaunay2D::opposite_vertex(int ti, int nb) const {

    for (int k = 0; k < 3; ++k)
        if (triangles[ti].adj[k] == nb)
            return triangles[ti].v[k];

    return -1;
}

int Delaunay2D::find_coincident(int start, double x, double y) const {

    if (start < 0 || !triangles[start].alive)
        return -1;

    for (int k = 0; k < 3; ++k) {
        const int vi = triangles[start].v[k];
        const double ddx = vertices[vi].x - x;
        const double ddy = vertices[vi].y - y;

        if (ddx * ddx + ddy * ddy < 1e-12)
            return vi;
    }

    return -1;
}

bool Delaunay2D::circumcircle_contains(int ti, double x, double y) const {

    const Triangle& tri = triangles[ti];
    const Vertex2D& a = vertices[tri.v[0]];
    const Vertex2D& b = vertices[tri.v[1]];
    const Vertex2D& c = vertices[tri.v[2]];
    const double o = orient2d(a.x, a.y, b.x, b.y, c.x, c.y);
    const double ic = (o > 0) ? in_circumcircle(a.x, a.y, b.x, b.y, c.x, c.y, x, y)
                              : in_circumcircle(a.x, a.y, c.x, c.y, b.x, b.y, x, y);

    return ic > 0;
}

void Delaunay2D::collect_cavity(int start, double x, double y) {

    ++visit_epoch_;

    if ((int)visit_stamp_.size() < (int)triangles.size() + 64)
        visit_stamp_.resize(triangles.size() + 64, 0);

    bad_.clear();

    if (start >= 0) {
        bad_.push_back(start);
        visit_stamp_[start] = visit_epoch_;
    }

    for (size_t front = 0; front < bad_.size(); ++front) {
        const int ti = bad_[front];

        if (!triangles[ti].alive || !circumcircle_contains(ti, x, y)) {
            bad_[front] = -1;
            continue;
        }

        for (int k = 0; k < 3; ++k) {
            const int nb = triangles[ti].adj[k];

            if (triangles[ti].constrained[k] || nb < 0 || visit_stamp_[nb] == visit_epoch_)
                continue;

            visit_stamp_[nb] = visit_epoch_;
            bad_.push_back(nb);
        }
    }

    std::erase(bad_, -1);
}

void Delaunay2D::cavity_polygon() {

    polygon_.clear();

    for (int ti : bad_) {
        const Triangle& tri = triangles[ti];

        for (int k = 0; k < 3; ++k) {
            const int nb = tri.adj[k];

            if (nb < 0 || std::find(bad_.begin(), bad_.end(), nb) == bad_.end())
                polygon_.push_back({tri.v[(k + 1) % 3], tri.v[(k + 2) % 3], tri.constrained[k]});
        }
    }
}

void Delaunay2D::fill_cavity(int vi) {

    for (int ti : bad_) {
        unregister_edges(ti);
        triangles[ti].alive = false;
    }

    for (const BEdge& edge : polygon_) {
        const double o = orient2d(
            vertices[vi].x,
            vertices[vi].y,
            vertices[edge.e0].x,
            vertices[edge.e0].y,
            vertices[edge.e1].x,
            vertices[edge.e1].y
        );

        if (std::abs(o) < 1e-20)
            continue;

        const int new_ti = (int)triangles.size();
        Triangle nt;
        nt.v[0] = vi;

        if (o > 0) {
            nt.v[1] = edge.e0;
            nt.v[2] = edge.e1;
        } else {
            nt.v[1] = edge.e1;
            nt.v[2] = edge.e0;
        }

        nt.constrained[0] = edge.constrained;
        triangles.push_back(nt);
        register_edges(new_ti);
    }
}

void Delaunay2D::insert_constraint(int v0, int v1) {

    if (v0 == v1 || constrain_existing(v0, v1))
        return;

    const int start_ti = first_triangle_at(v0);

    if (start_ti < 0)
        return;

    int ivl = -1;
    int ivr = -1;
    const int it = first_crossed(start_ti, v0, v1, ivl, ivr);

    if (it < 0)
        return;

    std::vector<int> poly_l = {v0, ivl};
    std::vector<int> poly_r = {v0, ivr};
    std::vector<int> intersected = {it};
    walk_crossed(v0, v1, ivl, ivr, poly_l, poly_r, intersected);
    poly_l.push_back(v1);
    poly_r.push_back(v1);
    retriangulate(v0, v1, poly_l, poly_r, intersected);
}

bool Delaunay2D::constrain_existing(int v0, int v1) {

    for (int ti = 0; ti < (int)triangles.size(); ++ti) {
        if (!triangles[ti].alive)
            continue;

        for (int k = 0; k < 3; ++k) {
            const int e0 = triangles[ti].v[(k + 1) % 3];
            const int e1 = triangles[ti].v[(k + 2) % 3];

            if (!((e0 == v0 && e1 == v1) || (e0 == v1 && e1 == v0)))
                continue;

            triangles[ti].constrained[k] = true;
            const int nb = triangles[ti].adj[k];

            if (nb >= 0 && triangles[nb].alive)
                for (int kk = 0; kk < 3; ++kk)
                    if (triangles[nb].adj[kk] == ti) {
                        triangles[nb].constrained[kk] = true;
                        break;
                    }

            return true;
        }
    }

    return false;
}

int Delaunay2D::first_triangle_at(int v) const {

    for (int ti = 0; ti < (int)triangles.size(); ++ti)
        if (triangles[ti].alive && has_vertex(ti, v))
            return ti;

    return -1;
}

int Delaunay2D::first_crossed(int start_ti, int v0, int v1, int& ivl, int& ivr) const {

    const Vertex2D& a = vertices[v0];
    const Vertex2D& b = vertices[v1];
    const int walk_guard = (int)triangles.size() + 4;
    int ti = start_ti;

    for (int g = 0; g < walk_guard && triangles[ti].alive; ++g) {
        const Triangle& t = triangles[ti];
        const int k_v0 = vertex_index(ti, v0);

        if (k_v0 < 0)
            return -1;

        const int ip2 = t.v[(k_v0 + 1) % 3];
        const int ip1 = t.v[(k_v0 + 2) % 3];
        const double op2 = orient2d(a.x, a.y, b.x, b.y, vertices[ip2].x, vertices[ip2].y);
        const double op1 = orient2d(a.x, a.y, b.x, b.y, vertices[ip1].x, vertices[ip1].y);

        if (op2 < 0 && op1 >= 0) {
            ivl = ip1;
            ivr = ip2;

            return ti;
        }

        const int next = t.adj[(k_v0 + 1) % 3];

        if (next < 0 || !triangles[next].alive || next == start_ti)
            return -1;

        ti = next;
    }

    return -1;
}

void Delaunay2D::walk_crossed(
    int v0,
    int v1,
    int ivl,
    int ivr,
    std::vector<int>& poly_l,
    std::vector<int>& poly_r,
    std::vector<int>& intersected
) const {

    const Vertex2D& a = vertices[v0];
    const Vertex2D& b = vertices[v1];
    const int cross_guard = (int)triangles.size() * 2 + 8;
    int iv = v0;
    int cur_it = intersected[0];

    for (int g = 0; g < cross_guard && !has_vertex(cur_it, v1); ++g) {
        const int k_iv = vertex_index(cur_it, iv);

        if (k_iv < 0)
            break;

        const int i_topo = triangles[cur_it].adj[k_iv];

        if (i_topo < 0 || !triangles[i_topo].alive)
            break;

        const int i_vopo = opposite_vertex(i_topo, cur_it);

        if (i_vopo < 0)
            break;

        const double o = orient2d(a.x, a.y, b.x, b.y, vertices[i_vopo].x, vertices[i_vopo].y);

        if (o < 0) {
            if (i_vopo != v1)
                poly_r.push_back(i_vopo);

            iv = ivr;
            ivr = i_vopo;
        } else {
            if (i_vopo != v1)
                poly_l.push_back(i_vopo);

            iv = ivl;
            ivl = i_vopo;
        }

        intersected.push_back(i_topo);
        cur_it = i_topo;
    }
}

void Delaunay2D::retriangulate(
    int v0,
    int v1,
    const std::vector<int>& poly_l,
    const std::vector<int>& poly_r,
    const std::vector<int>& intersected
) {

    for (int ti : intersected) {
        unregister_edges(ti);
        triangles[ti].alive = false;
    }

    const int first_new = (int)triangles.size();

    for (int i = 0; i + 2 < (int)poly_l.size(); ++i)
        add_triangle(v1, poly_l[i + 1], poly_l[i]);

    for (int i = 1; i + 1 < (int)poly_r.size(); ++i)
        add_triangle(v0, poly_r[i], poly_r[i + 1]);

    inherit_constraints(first_new);
    mark_edge(v0, v1);
}

void Delaunay2D::inherit_constraints(int first_new) {

    for (int new_ti = first_new; new_ti < (int)triangles.size(); ++new_ti) {
        if (!triangles[new_ti].alive)
            continue;

        Triangle& nt = triangles[new_ti];

        for (int k = 0; k < 3; ++k) {
            const int nb = nt.adj[k];

            if (nb < 0 || nb >= first_new || !triangles[nb].alive)
                continue;

            const Triangle& nb_t = triangles[nb];

            for (int kk = 0; kk < 3; ++kk)
                if (nb_t.adj[kk] == new_ti && nb_t.constrained[kk]) {
                    nt.constrained[k] = true;
                    break;
                }
        }
    }
}

void Delaunay2D::mark_edge(int v0, int v1) {

    for (Triangle& tri : triangles) {
        if (!tri.alive)
            continue;

        for (int k = 0; k < 3; ++k) {
            const int e0 = tri.v[(k + 1) % 3];
            const int e1 = tri.v[(k + 2) % 3];

            if ((e0 == v0 && e1 == v1) || (e0 == v1 && e1 == v0))
                tri.constrained[k] = true;
        }
    }
}

/// New counter-clockwise triangle over three vertices; skipped when degenerate.
void Delaunay2D::add_triangle(int pa, int pb, int pc) {

    const double o = orient2d(vertices[pa].x, vertices[pa].y, vertices[pb].x, vertices[pb].y, vertices[pc].x, vertices[pc].y);

    if (std::abs(o) < 1e-20)
        return;

    Triangle nt;
    nt.v[0] = pa;

    if (o > 0) {
        nt.v[1] = pb;
        nt.v[2] = pc;
    } else {
        nt.v[1] = pc;
        nt.v[2] = pb;
    }

    const int new_ti = (int)triangles.size();
    triangles.push_back(nt);
    register_edges(new_ti);
}

void Delaunay2D::cleanup() {

    for (Triangle& tri : triangles) {
        if (!tri.alive)
            continue;

        for (int k = 0; k < 3; ++k)
            if (tri.v[k] == super_v[0] || tri.v[k] == super_v[1] || tri.v[k] == super_v[2]) {
                unregister_edges((int)(&tri - &triangles[0]));
                tri.alive = false;
                break;
            }
    }

    last_found = 0;

    for (int i = 0; i < (int)triangles.size(); ++i)
        if (triangles[i].alive) {
            last_found = i;
            break;
        }
}

std::vector<std::array<int, 3>> Delaunay2D::get_triangles() const {

    std::vector<std::array<int, 3>> result;

    for (const Triangle& tri : triangles) {
        if (!tri.alive)
            continue;

        double o = orient2d(
            vertices[tri.v[0]].x,
            vertices[tri.v[0]].y,
            vertices[tri.v[1]].x,
            vertices[tri.v[1]].y,
            vertices[tri.v[2]].x,
            vertices[tri.v[2]].y
        );
        result.push_back(
            o > 0 ? std::array<int, 3>{tri.v[0], tri.v[1], tri.v[2]} : std::array<int, 3>{tri.v[0], tri.v[2], tri.v[1]}
        );
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// Triangulation
// ═══════════════════════════════════════════════════════════════════════════
/// Loop polygon in UV before refinement: the control points of a polyline, else samples, the closing repeat dropped.
std::vector<Point> loop_points(const NurbsCurve& crv) {

    std::vector<Point> raw;

    if (crv.degree() <= 1 && !crv.is_rational()) {
        for (int i = 0; i < crv.cv_count(); ++i)
            raw.push_back(crv.get_cv(i));
    } else {
        const int n = std::min(std::max(crv.cv_count() * 4, 16), 2048);
        raw = crv.divide_by_count(n).first;
    }

    while (raw.size() > 1) {
        const double dx = raw.front()[0] - raw.back()[0];
        const double dy = raw.front()[1] - raw.back()[1];

        if (dx * dx + dy * dy < 1e-20)
            raw.pop_back();
        else
            break;
    }

    return raw;
}

/// Append the UV points of the edge start-end without end, halved up to six times until each 3D chord is within deflection.
void subdivide_edge(
    const NurbsSurface& srf,
    const Point& start,
    const Point& end,
    double deflection,
    std::vector<Point>& out
) {

    std::vector<std::tuple<Point, Point, int>> stack = {{start, end, 0}};

    while (!stack.empty()) {
        Point a;
        Point b;
        int depth;
        std::tie(a, b, depth) = stack.back();
        stack.pop_back();
        const double mu = (a[0] + b[0]) * 0.5;
        const double mv = (a[1] + b[1]) * 0.5;
        const Point pa = srf.point_at(a[0], a[1]);
        const Point pm = srf.point_at(mu, mv);
        const Vector edge = srf.point_at(b[0], b[1]) - pa;
        const double l2 = edge.magnitude_squared();
        double dev = 0.0;

        if (l2 > 1e-30) {
            const double t = (pm - pa).dot(edge) / l2;
            dev = std::sqrt((pm - (pa + edge * t)).magnitude_squared());
        } else {
            dev = std::sqrt((pm - pa).magnitude_squared());
        }

        if (dev > deflection && depth < 6) {
            stack.push_back({Point(mu, mv, 0.0), b, depth + 1});
            stack.push_back({a, Point(mu, mv, 0.0), depth + 1});
        } else {
            out.push_back(a);
        }
    }
}

/// Interior knots per direction whose multiplicity reaches the degree: the C0 lines of the surface.
std::array<std::vector<double>, 2> find_crease_knots(const NurbsSurface& surface) {

    std::array<std::vector<double>, 2> crease_knots;

    for (int dir = 0; dir < 2; ++dir) {
        const std::pair<double, double> domain = surface.domain(dir);
        const std::vector<double>& knots = surface.m_nurbsknot[dir];

        for (double knot : knots) {
            if (knot <= domain.first || knot >= domain.second ||
                std::find(crease_knots[dir].begin(), crease_knots[dir].end(), knot) != crease_knots[dir].end())
                continue;

            if (std::count(knots.begin(), knots.end(), knot) >= surface.degree(dir))
                crease_knots[dir].push_back(knot);
        }
    }

    return crease_knots;
}

/// UV bounds (umin, vmin, umax, vmax) of a loop polygon.
std::array<double, 4> loop_bounds(const std::vector<Point>& pts) {

    std::array<double, 4> bounds = {1e30, 1e30, -1e30, -1e30};

    for (const Point& p : pts) {
        if (p[0] < bounds[0])
            bounds[0] = p[0];

        if (p[1] < bounds[1])
            bounds[1] = p[1];

        if (p[0] > bounds[2])
            bounds[2] = p[0];

        if (p[1] > bounds[3])
            bounds[3] = p[1];
    }

    return bounds;
}

/// Constrain loop edge i in pieces cut where it crosses a crease knot line, each crossing inserted and recorded.
void insert_loop_edge(
    Delaunay2D& dt,
    const std::vector<Point>& pts,
    const std::vector<int>& vis,
    size_t li,
    size_t i,
    const std::array<std::vector<double>, 2>& crease_knots,
    std::map<int, std::tuple<size_t, size_t, double>>& boundary_intervals
) {

    const size_t j = (i + 1) % vis.size();
    std::vector<std::pair<double, int>> events = {{0.0, vis[i]}, {1.0, vis[j]}};

    for (int dir = 0; dir < 2; ++dir) {
        const double delta = pts[j][dir] - pts[i][dir];

        if (delta == 0.0)
            continue;

        for (double knot : crease_knots[dir]) {
            const double t = (knot - pts[i][dir]) / delta;

            if (t <= 0.0 || t >= 1.0)
                continue;

            double uv[2] = {pts[i][0] + t * (pts[j][0] - pts[i][0]), pts[i][1] + t * (pts[j][1] - pts[i][1])};
            uv[dir] = knot;
            const int vi = dt.insert(uv[0], uv[1]);

            if (vi >= 0)
                boundary_intervals[vi] = {li, i, t};

            events.push_back({t, vi});
        }
    }

    std::sort(events.begin(), events.end());

    for (size_t k = 1; k < events.size(); ++k)
        if (events[k - 1].second >= 0 && events[k].second >= 0 && events[k - 1].second != events[k].second)
            dt.insert_constraint(events[k - 1].second, events[k].second);
}

/// Insert each loop's vertices, then constrain its edges; the vertex index of every loop sample.
std::vector<std::vector<int>> insert_loops(
    Delaunay2D& dt,
    const std::vector<std::vector<Point>>& loops_uv,
    const std::array<std::vector<double>, 2>& crease_knots,
    std::map<int, std::tuple<size_t, size_t, double>>& boundary_intervals
) {

    std::vector<std::vector<int>> loop_vids;

    for (size_t li = 0; li < loops_uv.size(); ++li) {
        std::vector<int> vis;

        for (const Point& p : loops_uv[li])
            vis.push_back(dt.insert(p[0], p[1]));

        for (size_t i = 0; i < vis.size(); ++i)
            insert_loop_edge(dt, loops_uv[li], vis, li, i, crease_knots, boundary_intervals);

        loop_vids.push_back(vis);
    }

    return loop_vids;
}

/// Insert the crease knot crossings inside the loops and constrain each knot line between consecutive vertices on it.
void insert_crease_lines(
    Delaunay2D& dt,
    const std::vector<std::vector<Point>>& loops_uv,
    const std::array<std::vector<double>, 2>& crease_knots
) {

    for (double u : crease_knots[0])
        for (double v : crease_knots[1])
            if (inside_loops(u, v, loops_uv))
                dt.insert(u, v);

    for (int dir = 0; dir < 2; ++dir) {
        for (double knot : crease_knots[dir]) {
            std::vector<std::pair<double, int>> nodes;

            for (size_t vi = 0; vi < dt.vertices.size(); ++vi) {
                const double uv[2] = {dt.vertices[vi].x, dt.vertices[vi].y};

                if (uv[dir] == knot)
                    nodes.push_back({uv[1 - dir], (int)vi});
            }

            std::sort(nodes.begin(), nodes.end());

            for (size_t k = 1; k < nodes.size(); ++k) {
                double uv[2] = {knot, knot};
                uv[1 - dir] = (nodes[k - 1].first + nodes[k].first) * 0.5;

                if (inside_loops(uv[0], uv[1], loops_uv))
                    dt.insert_constraint(nodes[k - 1].second, nodes[k].second);
            }
        }
    }
}

/// Smallest dot product between the crease-side normals at the corners of triangle ABC around its centroid.
double min_normal_dot(
    const NurbsSurface& surface,
    const std::array<std::vector<double>, 2>& crease_knots,
    const std::array<double, 2>& center,
    const Vertex2D& A,
    const Vertex2D& B,
    const Vertex2D& C
) {

    const Vector na = crease_side_normal(surface, crease_knots, center, {A.x, A.y});
    const Vector nb = crease_side_normal(surface, crease_knots, center, {B.x, B.y});
    const Vector nc2 = crease_side_normal(surface, crease_knots, center, {C.x, C.y});
    const double d1 = na.dot(nb);
    const double d2 = nb.dot(nc2);
    const double d3 = na.dot(nc2);

    return std::min(d1, std::min(d2, d3));
}

/// Centroids of the live triangles inside the loops whose chord leaves deflection or whose corner normals turn past the angle bound.
std::vector<std::array<double, 2>> refinement_points(
    const Delaunay2D& dt,
    const NurbsSurface& surface,
    const std::vector<std::vector<Point>>& loops_uv,
    const std::array<std::vector<double>, 2>& crease_knots,
    double deflection,
    double cos_max_angle
) {

    std::vector<std::array<double, 2>> to_insert;

    for (const Triangle& tri : dt.triangles) {
        if (!tri.alive)
            continue;

        const Vertex2D& A = dt.vertices[tri.v[0]];
        const Vertex2D& B = dt.vertices[tri.v[1]];
        const Vertex2D& C = dt.vertices[tri.v[2]];
        const double cu = (A.x + B.x + C.x) / 3.0;
        const double cv = (A.y + B.y + C.y) / 3.0;

        if (!inside_loops(cu, cv, loops_uv))
            continue;

        const Point pa = surface.point_at(A.x, A.y);
        const Point pb = surface.point_at(B.x, B.y);
        const Point pc = surface.point_at(C.x, C.y);
        const Point pm = surface.point_at(cu, cv);
        const Vector n = (pb - pa).cross(pc - pa);
        const double nl = std::sqrt(n.magnitude_squared());

        if (nl < 1e-30)
            continue;

        const double dev = std::abs((pm - pa).dot(n) / nl);

        if (dev > deflection || min_normal_dot(surface, crease_knots, {cu, cv}, A, B, C) < cos_max_angle)
            to_insert.push_back({cu, cv});
    }

    return to_insert;
}

/// Insert refinement centroids for up to eight rounds, until none is needed or the vertex cap is hit.
void refine(
    Delaunay2D& dt,
    const NurbsSurface& surface,
    const std::vector<std::vector<Point>>& loops_uv,
    const std::array<std::vector<double>, 2>& crease_knots,
    double deflection,
    double cos_max_angle
) {

    const int MAX_ITERS = 8;
    const size_t MAX_VERTS = 200000;

    for (int iter = 0; iter < MAX_ITERS; ++iter) {
        const std::vector<std::array<double, 2>> to_insert =
            refinement_points(dt, surface, loops_uv, crease_knots, deflection, cos_max_angle);

        if (to_insert.empty())
            break;

        for (const std::array<double, 2>& uv : to_insert) {
            if (dt.vertices.size() >= MAX_VERTS)
                break;

            dt.insert(uv[0], uv[1]);
        }

        if (dt.vertices.size() >= MAX_VERTS)
            break;
    }
}

/// Drop the super triangle and every triangle whose centroid lies outside the loops.
void trim_outside(Delaunay2D& dt, const std::vector<std::vector<Point>>& loops_uv) {

    dt.cleanup();

    for (Triangle& tri : dt.triangles) {
        if (!tri.alive)
            continue;

        const double cu = (dt.vertices[tri.v[0]].x + dt.vertices[tri.v[1]].x + dt.vertices[tri.v[2]].x) / 3.0;
        const double cv = (dt.vertices[tri.v[0]].y + dt.vertices[tri.v[1]].y + dt.vertices[tri.v[2]].y) / 3.0;

        if (!inside_loops(cu, cv, loops_uv))
            tri.alive = false;
    }
}

/// True when a triangle spans a crease knot line in either direction.
bool crosses_crease(
    const std::vector<std::array<int, 3>>& tris,
    const Delaunay2D& dt,
    const std::array<std::vector<double>, 2>& crease_knots
) {

    for (const std::array<int, 3>& tri : tris)
        for (int dir = 0; dir < 2; ++dir) {
            double low = INFINITY;
            double high = -INFINITY;

            for (int vi : tri) {
                const double value = dir == 0 ? dt.vertices[vi].x : dt.vertices[vi].y;
                low = std::min(low, value);
                high = std::max(high, value);
            }

            for (double knot : crease_knots[dir])
                if (low < knot && knot < high)
                    return true;
        }

    return false;
}

/// Loop and sample of the 3D point given for each triangulation vertex, (-1, -1) where none is.
std::vector<std::pair<int, int>> given_points(
    size_t count,
    const TrimLoops& loops,
    const std::vector<std::vector<int>>& loop_vids
) {

    std::vector<std::pair<int, int>> given(count, {-1, -1});

    for (size_t li = 0; li < loop_vids.size() && li < loops.xyz.size(); ++li)
        for (size_t k = 0; k < loop_vids[li].size() && k < loops.xyz[li].size(); ++k)
            if (loop_vids[li][k] >= 0)
                given[loop_vids[li][k]] = {(int)li, (int)k};

    return given;
}

/// 3D point of triangulation vertex vi: its given loop point, the loop chord at a knot crossing, else the surface point.
Point vertex_point(
    const NurbsSurface& surface,
    const Delaunay2D& dt,
    int vi,
    const TrimLoops& loops,
    const std::vector<std::pair<int, int>>& given,
    const std::map<int, std::tuple<size_t, size_t, double>>& boundary_intervals
) {

    if (given[vi].first >= 0)
        return loops.xyz[given[vi].first][given[vi].second];

    const auto interval = boundary_intervals.find(vi);

    if (interval != boundary_intervals.end() && !loops.xyz.empty()) {
        size_t li;
        size_t segment;
        double t;
        std::tie(li, segment, t) = interval->second;
        const Point& a = loops.xyz[li][segment];
        const Point& b = loops.xyz[li][(segment + 1) % loops.xyz[li].size()];

        return a + (b - a) * t;
    }

    return surface.point_at(dt.vertices[vi].x, dt.vertices[vi].y);
}

/// Welded mesh vertex of every triangulation vertex a triangle uses, SIZE_MAX for the others.
std::vector<size_t> weld_vertices(
    VertexWelder& welder,
    const NurbsSurface& surface,
    const Delaunay2D& dt,
    const std::vector<std::array<int, 3>>& tris,
    const TrimLoops& loops,
    const std::vector<std::vector<int>>& loop_vids,
    const std::map<int, std::tuple<size_t, size_t, double>>& boundary_intervals
) {

    const std::vector<std::pair<int, int>> given = given_points(dt.vertices.size(), loops, loop_vids);
    std::vector<size_t> vert_map(dt.vertices.size(), SIZE_MAX);

    for (const std::array<int, 3>& tri : tris)
        for (int vi : tri)
            if (vert_map[vi] == SIZE_MAX)
                vert_map[vi] = welder.weld(vertex_point(surface, dt, vi, loops, given, boundary_intervals));

    return vert_map;
}

/// One face per triangle over its welded vertices, collapsed ones skipped.
void add_faces(Mesh& mesh, const std::vector<std::array<int, 3>>& tris, const std::vector<size_t>& vert_map) {

    for (const std::array<int, 3>& tri : tris) {
        const size_t v0 = vert_map[tri[0]];
        const size_t v1 = vert_map[tri[1]];
        const size_t v2 = vert_map[tri[2]];

        if (v0 == v1 || v1 == v2 || v2 == v0)
            continue;

        mesh.add_face({v0, v1, v2});
    }
}

/// Area-weighted sum of the face normals around each mesh vertex.
std::map<size_t, Vector> fan_normals(const Mesh& mesh) {

    std::map<size_t, Vector> fan;

    for (const std::pair<const size_t, std::vector<size_t>>& entry : mesh.face) {
        const std::vector<size_t>& verts = entry.second;
        const Point a = mesh.vertex.at(verts[0]).position();
        const Point b = mesh.vertex.at(verts[1]).position();
        const Point c = mesh.vertex.at(verts[2]).position();
        const Vector n = (b - a).cross(c - a);

        for (size_t vk : verts)
            fan[vk] += n;
    }

    return fan;
}

/// Normal of every used vertex from the surface derivatives, the fan normal where they degenerate, and its u and v.
void set_vertex_normals(
    Mesh& mesh,
    const NurbsSurface& surface,
    const Delaunay2D& dt,
    const std::vector<size_t>& vert_map
) {

    const std::map<size_t, Vector> fan = fan_normals(mesh);

    for (size_t vi = 0; vi < vert_map.size(); ++vi) {
        if (vert_map[vi] == SIZE_MAX)
            continue;

        VertexData& vd = mesh.vertex[vert_map[vi]];
        const std::vector<Vector> derivatives = surface.evaluate(dt.vertices[vi].x, dt.vertices[vi].y, 1);
        Vector nrm(0.0, 0.0, 0.0);

        if (derivatives.size() >= 3)
            nrm = derivatives[2].cross(derivatives[1]);

        const double nl = std::sqrt(nrm.magnitude_squared());

        if (std::isfinite(nl) && nl > 0.0) {
            nrm = nrm / nl;
        } else {
            const Vector f = fan.count(vert_map[vi]) ? fan.at(vert_map[vi]) : Vector(0.0, 0.0, 1.0);
            const double fl = std::sqrt(f.magnitude_squared());
            nrm = std::isfinite(fl) && fl > 0.0 ? f / fl : Vector(0.0, 0.0, 1.0);
        }

        vd.set_normal(nrm[0], nrm[1], nrm[2]);
        vd.attributes["u"] = dt.vertices[vi].x;
        vd.attributes["v"] = dt.vertices[vi].y;
    }
}

/// Tag loop vertices boundary/{loop}/{sample} and knot crossings boundary_interval/{loop}/{segment} with their chord parameter.
void tag_boundary(
    Mesh& mesh,
    const std::vector<std::vector<int>>& loop_vids,
    const std::map<int, std::tuple<size_t, size_t, double>>& boundary_intervals,
    const std::vector<size_t>& vert_map
) {

    for (size_t li = 0; li < loop_vids.size(); ++li) {
        for (size_t k = 0; k < loop_vids[li].size(); ++k) {
            const int vi = loop_vids[li][k];
            const std::string key = "boundary/" + std::to_string(li) + "/" + std::to_string(k);

            if (vi >= 0 && vert_map[vi] != SIZE_MAX)
                mesh.vertex[vert_map[vi]].attributes[key] = 1.0;
        }
    }

    for (const std::pair<const int, std::tuple<size_t, size_t, double>>& entry : boundary_intervals) {
        const size_t li = std::get<0>(entry.second);
        const size_t segment = std::get<1>(entry.second);
        const std::string key = "boundary_interval/" + std::to_string(li) + "/" + std::to_string(segment);

        if (vert_map[entry.first] != SIZE_MAX)
            mesh.vertex[vert_map[entry.first]].attributes[key] = std::get<2>(entry.second);
    }
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Constructors
// ═══════════════════════════════════════════════════════════════════════════
NurbsSurfaceTrimmed::NurbsSurfaceTrimmed() {}

NurbsSurfaceTrimmed::NurbsSurfaceTrimmed(const NurbsSurfaceTrimmed& other) {
    deep_copy_from(other);
}

NurbsSurfaceTrimmed& NurbsSurfaceTrimmed::operator=(const NurbsSurfaceTrimmed& other) {

    if (this != &other)
        deep_copy_from(other);

    return *this;
}

NurbsSurfaceTrimmed::~NurbsSurfaceTrimmed() {}

// ═══════════════════════════════════════════════════════════════════════════
// Static constructors
// ═══════════════════════════════════════════════════════════════════════════
NurbsSurfaceTrimmed NurbsSurfaceTrimmed::create(const NurbsSurface& surface, const NurbsCurve& outer_loop) {

    NurbsSurfaceTrimmed ts;
    ts.m_surface = surface;
    ts.m_outer_loop = outer_loop;

    return ts;
}

NurbsSurfaceTrimmed NurbsSurfaceTrimmed::create_planar(const NurbsCurve& boundary) {

    NurbsSurface srf = Primitives::create_planar(boundary);

    if (!srf.is_valid())
        return NurbsSurfaceTrimmed();

    const Point p00 = srf.get_cv(0, 0);
    const Vector u_axis = srf.get_cv(1, 0) - p00;
    const Vector v_axis = srf.get_cv(0, 1) - p00;
    const double u_len2 = u_axis.magnitude_squared();
    const double v_len2 = v_axis.magnitude_squared();

    if (u_len2 < 1e-28 || v_len2 < 1e-28)
        return NurbsSurfaceTrimmed();

    std::vector<Point> uv_pts;

    if (boundary.degree() <= 1) {
        for (int i = 0; i < boundary.cv_count(); ++i)
            uv_pts.push_back(project_to_uv(boundary.get_cv(i), p00, u_axis, v_axis, u_len2, v_len2));
    } else {
        const std::vector<double> spans = boundary.get_span_vector();

        const int n_sub = 10;

        for (size_t si = 0; si + 1 < spans.size(); ++si)
            for (int k = 0; k <= n_sub; ++k) {
                const double t = spans[si] + (spans[si + 1] - spans[si]) * k / n_sub;
                const Point uv = project_to_uv(boundary.point_at(t), p00, u_axis, v_axis, u_len2, v_len2);

                if (uv_pts.empty() || (uv - uv_pts.back()).magnitude_squared() > 1e-24)
                    uv_pts.push_back(uv);
            }
    }

    NurbsSurfaceTrimmed ts;
    ts.m_surface = srf;

    if (uv_pts.size() >= 3)
        ts.m_outer_loop = NurbsCurve::create(false, 1, uv_pts);

    return ts;
}

std::vector<NurbsSurfaceTrimmed> NurbsSurfaceTrimmed::split_by_uv_curves(
    const NurbsSurface& srf,
    const std::vector<NurbsCurve>& pcurves,
    double tolerance
) {

    if (!srf.is_valid())
        return {};

    const SplitDomain dom = split_domain(srf, tolerance);
    const std::vector<UVPoly> polylines = uv_polylines(pcurves, dom);
    UVVertexPool pool(dom.snap);
    const std::map<std::pair<int, int>, std::vector<std::array<double, 4>>> splits =
        polyline_crossings(polylines, pcurves, dom);
    const std::vector<SplitEdge> live_edges = prune_dangling(split_edges(polylines, splits, pool));

    if (live_edges.empty())
        return {};

    const std::vector<std::array<double, 2>>& verts = pool.verts;
    const std::vector<HalfEdge> hes = half_edges(live_edges);
    std::vector<std::pair<std::vector<int>, double>> pos_faces;
    std::vector<std::vector<int>> neg_faces;
    const std::vector<std::vector<int>> faces = face_cycles(next_half_edges(hes, verts));
    classify_faces(faces, hes, verts, live_edges, dom.snap, pos_faces, neg_faces);
    const std::vector<std::vector<std::vector<int>>> holes_of = assign_holes(neg_faces, pos_faces, hes, verts);
    std::vector<NurbsSurfaceTrimmed> result;

    for (int fi = 0; fi < (int)pos_faces.size(); ++fi) {
        NurbsCurve outer = cycle_to_loop(pos_faces[fi].first, hes, live_edges, verts, pcurves, dom.snap);

        if (!outer.is_valid() || (loop_signed_area(outer) < 0.0 && !outer.reverse()))
            continue;

        NurbsSurfaceTrimmed ts = NurbsSurfaceTrimmed::create(srf, outer);

        for (const std::vector<int>& hole_cycle : holes_of[fi]) {
            NurbsCurve hole = cycle_to_loop(hole_cycle, hes, live_edges, verts, pcurves, dom.snap);

            if (!hole.is_valid() || (loop_signed_area(hole) > 0.0 && !hole.reverse()))
                continue;

            ts.add_inner_loop(hole);
        }

        result.push_back(ts);
    }

    return result;
}

std::vector<NurbsSurfaceTrimmed> NurbsSurfaceTrimmed::split_by_planes(
    const NurbsSurface& srf,
    const std::vector<std::pair<Point, Vector>>& planes
) {

    std::vector<NurbsSurfaceTrimmed> out;
    const int k = (int)planes.size();

    if (k == 0 || k > 16)
        return out;

    for (int mask = 0; mask < (1 << k); ++mask) {
        std::vector<std::pair<Point, Vector>> cp;

        for (int i = 0; i < k; ++i) {
            const Point& q = planes[i].first;
            const Vector& n = planes[i].second;
            const bool flip = ((mask >> i) & 1) == 1;
            Vector nn = flip ? Vector(-n[0], -n[1], -n[2]) : Vector(n[0], n[1], n[2]);
            cp.push_back({q, nn});
        }

        NurbsSurfaceTrimmed ts;
        ts.m_surface = srf;
        Mesh m = ts.mesh_by_planes(cp, 20.0, 0.01);

        if (m.number_of_faces() > 0)
            out.push_back(ts);
    }

    return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
bool NurbsSurfaceTrimmed::operator==(const NurbsSurfaceTrimmed& other) const {

    if (name != other.name)
        return false;

    if (width != other.width)
        return false;

    if (surfacecolor != other.surfacecolor)
        return false;

    if (m_surface != other.m_surface)
        return false;

    if (m_outer_loop != other.m_outer_loop)
        return false;

    return m_inner_loops == other.m_inner_loops;
}

bool NurbsSurfaceTrimmed::operator!=(const NurbsSurfaceTrimmed& other) const {
    return !(*this == other);
}

// ═══════════════════════════════════════════════════════════════════════════
// Transformation
// ═══════════════════════════════════════════════════════════════════════════
void NurbsSurfaceTrimmed::transform(const Xform& xform) {
    m_surface.transform(xform);
}

NurbsSurfaceTrimmed NurbsSurfaceTrimmed::transformed(const Xform& xform) const {

    NurbsSurfaceTrimmed ts = *this;
    ts.transform(xform);

    return ts;
}

// ═══════════════════════════════════════════════════════════════════════════
// Accessors
// ═══════════════════════════════════════════════════════════════════════════
NurbsSurface NurbsSurfaceTrimmed::surface() const {
    return m_surface;
}

NurbsCurve NurbsSurfaceTrimmed::get_outer_loop() const {
    return m_outer_loop;
}

void NurbsSurfaceTrimmed::set_outer_loop(const NurbsCurve& loop) {
    m_outer_loop = loop;
}

bool NurbsSurfaceTrimmed::is_trimmed() const {
    return m_outer_loop.is_valid();
}

bool NurbsSurfaceTrimmed::is_valid() const {
    return m_surface.is_valid();
}

// ═══════════════════════════════════════════════════════════════════════════
// Inner loops
// ═══════════════════════════════════════════════════════════════════════════
void NurbsSurfaceTrimmed::add_inner_loop(const NurbsCurve& loop_2d) {
    m_inner_loops.push_back(loop_2d);
}

void NurbsSurfaceTrimmed::add_hole(const NurbsCurve& curve_3d) {

    const std::pair<double, double> dom = curve_3d.domain();
    const std::pair<double, double> sdom_u = m_surface.domain(0);
    const std::pair<double, double> sdom_v = m_surface.domain(1);
    const double range_u = sdom_u.second - sdom_u.first;
    const double range_v = sdom_v.second - sdom_v.first;
    const int n_samples = std::min(std::max(curve_3d.cv_count() * 4, 32), 2048);
    std::vector<Point> uv_pts;

    for (int i = 0; i < n_samples; ++i) {
        const double t = dom.first + (dom.second - dom.first) * i / n_samples;
        const Point pt3d = curve_3d.point_at(t);
        double u;
        double v;
        std::tie(u, v, std::ignore) = Closest::surface_point(m_surface, pt3d);
        const double nu = (u - sdom_u.first) / range_u;
        const double nv = (v - sdom_v.first) / range_v;
        uv_pts.push_back(Point(nu, nv, 0.0));
    }

    if (uv_pts.size() >= 3)
        m_inner_loops.push_back(NurbsCurve::create(true, 1, uv_pts));
}

void NurbsSurfaceTrimmed::add_holes(const std::vector<NurbsCurve>& curves_3d) {

    for (const NurbsCurve& crv : curves_3d)
        add_hole(crv);
}

NurbsCurve NurbsSurfaceTrimmed::get_inner_loop(int index) const {
    return m_inner_loops[index];
}

int NurbsSurfaceTrimmed::inner_loop_count() const {
    return static_cast<int>(m_inner_loops.size());
}

void NurbsSurfaceTrimmed::clear_inner_loops() {
    m_inner_loops.clear();
}

// ═══════════════════════════════════════════════════════════════════════════
// Evaluation
// ═══════════════════════════════════════════════════════════════════════════
Point NurbsSurfaceTrimmed::point_at(double u, double v) const {
    return m_surface.point_at(u, v);
}

Vector NurbsSurfaceTrimmed::normal_at(double u, double v) const {
    return m_surface.normal_at(u, v);
}

// ═══════════════════════════════════════════════════════════════════════════
// Meshing
// ═══════════════════════════════════════════════════════════════════════════
Mesh NurbsSurfaceTrimmed::mesh() const {
    return mesh_q(20.0, 0.005);
}

Mesh NurbsSurfaceTrimmed::mesh_q(double max_angle_deg, double chord_factor) const {

    if (!is_trimmed())
        return m_surface.mesh();

    const double deflection = bbox_diagonal() * chord_factor;

    TrimLoops loops;
    loops.uv.push_back(discretize_loop(m_outer_loop, deflection));

    for (const NurbsCurve& inner : m_inner_loops)
        loops.uv.push_back(discretize_loop(inner, deflection));

    return triangulate(loops, max_angle_deg, chord_factor);
}

Mesh NurbsSurfaceTrimmed::mesh_loops(const TrimLoops& loops, double max_angle_deg, double chord_factor) const {

    if (loops.uv.empty() || !std::isfinite(max_angle_deg) || max_angle_deg <= 0.0 || !std::isfinite(chord_factor) ||
        chord_factor <= 0.0 || (!loops.xyz.empty() && loops.xyz.size() != loops.uv.size()))

        return Mesh();

    size_t expected = 0;

    for (size_t li = 0; li < loops.uv.size(); ++li) {
        const std::vector<Point>& points = loops.uv[li];

        if (points.size() < 3 || (!loops.xyz.empty() && loops.xyz[li].size() != points.size()))
            return Mesh();

        for (const Point& point : points)
            if (!std::isfinite(point[0]) || !std::isfinite(point[1]))
                return Mesh();

        if (!loops.xyz.empty())
            for (const Point& point : loops.xyz[li])
                if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2]))
                    return Mesh();

        expected += points.size();
    }

    Mesh result = triangulate(loops, max_angle_deg, chord_factor);
    std::set<std::string> actual;

    for (const std::pair<const size_t, VertexData>& entry : result.vertex)
        for (const std::pair<const std::string, double>& attribute : entry.second.attributes)
            if (attribute.first.starts_with("boundary/"))
                actual.insert(attribute.first);

    return actual.size() == expected ? result : Mesh();
}

Mesh NurbsSurfaceTrimmed::mesh_by_plane(
    const Point& q0,
    const Vector& normal,
    double max_angle_deg,
    double chord_factor
) const {

    const NurbsSurface& srf = m_surface;
    std::array<double, 3> n;

    if (!unit3(normal, n))
        return srf.mesh();

    const std::array<double, 3> q = {q0[0], q0[1], q0[2]};
    const double bbox_diag = bbox_diagonal();
    std::vector<double> us;
    std::vector<double> vs;

    if (!span_grid(srf, max_angle_deg, bbox_diag * chord_factor, us, vs))
        return srf.mesh();

    const int nu = (int)us.size();
    const int nv = (int)vs.size();
    std::vector<std::vector<double>> field(nu, std::vector<double>(nv));

    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j)
            field[i][j] = plane_field(srf, q, n, us[i], vs[j]);

    Mesh result;
    const double weld_tol = bbox_diag * 1e-5;
    VertexWelder welder(result, weld_tol, weld_tol);

    for (int i = 0; i + 1 < nu; ++i) {
        for (int j = 0; j + 1 < nv; ++j) {
            const std::array<double, 4> cu = {us[i], us[i + 1], us[i + 1], us[i]};
            const std::array<double, 4> cv = {vs[j], vs[j], vs[j + 1], vs[j + 1]};
            const std::array<double, 4> fc = {field[i][j], field[i + 1][j], field[i + 1][j + 1], field[i][j + 1]};
            add_fan(result, clip_cell(welder, srf, q, n, cu, cv, fc));
        }
    }

    if (result.face.empty())
        return srf.mesh();

    return result;
}

Mesh NurbsSurfaceTrimmed::mesh_by_planes(
    const std::vector<std::pair<Point, Vector>>& planes,
    double max_angle_deg,
    double chord_factor
) const {

    const NurbsSurface& srf = m_surface;
    std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>> pl;

    for (const std::pair<Point, Vector>& qn : planes) {
        std::array<double, 3> n;

        if (!unit3(qn.second, n))
            continue;

        pl.push_back({{qn.first[0], qn.first[1], qn.first[2]}, n});
    }

    if (pl.empty())
        return srf.mesh();

    const double bbox_diag = bbox_diagonal();
    std::vector<double> us;
    std::vector<double> vs;

    if (!span_grid(srf, max_angle_deg, bbox_diag * chord_factor, us, vs))
        return srf.mesh();

    std::vector<std::array<std::array<double, 2>, 3>> tris = grid_triangles(us, vs);

    for (const std::pair<std::array<double, 3>, std::array<double, 3>>& plane : pl) {
        tris = clip_triangles(srf, plane.first, plane.second, tris);

        if (tris.empty())
            break;
    }

    if (tris.empty())
        return Mesh();

    const Mesh result = weld_triangles(srf, tris, bbox_diag * 1e-5);

    return result.face.empty() ? Mesh() : result;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON
// ═══════════════════════════════════════════════════════════════════════════
nlohmann::ordered_json NurbsSurfaceTrimmed::jsondump() const {

    nlohmann::ordered_json j;
    j["guid"] = guid();
    j["inner_loops"] = nlohmann::ordered_json::array();

    for (const NurbsCurve& loop : m_inner_loops)
        j["inner_loops"].push_back(loop.jsondump());

    j["name"] = name;

    if (m_outer_loop.is_valid())
        j["outer_loop"] = m_outer_loop.jsondump();

    j["surface"] = m_surface.jsondump();
    j["surfacecolor"] = surfacecolor.jsondump();
    j["type"] = "NurbsSurfaceTrimmed";
    j["width"] = width;

    return j;
}

NurbsSurfaceTrimmed NurbsSurfaceTrimmed::jsonload(const nlohmann::json& data) {

    NurbsSurfaceTrimmed ts;

    if (data.contains("guid"))
        ts.guid() = data["guid"];

    if (data.contains("name"))
        ts.name = data["name"];

    if (data.contains("width"))
        ts.width = data["width"];

    if (data.contains("surfacecolor"))
        ts.surfacecolor = Color::jsonload(data["surfacecolor"]);

    if (data.contains("surface"))
        ts.m_surface = NurbsSurface::jsonload(data["surface"]);

    if (data.contains("outer_loop"))
        ts.m_outer_loop = NurbsCurve::jsonload(data["outer_loop"]);

    if (data.contains("inner_loops"))
        for (const nlohmann::json& loop_data : data["inner_loops"])
            ts.m_inner_loops.push_back(NurbsCurve::jsonload(loop_data));

    return ts;
}

std::string NurbsSurfaceTrimmed::file_json_dumps() const {
    return jsondump().dump();
}

NurbsSurfaceTrimmed NurbsSurfaceTrimmed::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void NurbsSurfaceTrimmed::file_json_dump(const std::string& filename) const {

    std::ofstream file(filename);
    file << jsondump().dump(4);
}

NurbsSurfaceTrimmed NurbsSurfaceTrimmed::file_json_load(const std::string& filename) {

    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;

    return jsonload(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// Protobuf
// ═══════════════════════════════════════════════════════════════════════════
session_proto::NurbsSurfaceTrimmed NurbsSurfaceTrimmed::to_proto() const {

    session_proto::NurbsSurfaceTrimmed proto;

    if (has_guid())
        proto.set_guid(guid());

    proto.set_name(name);
    proto.set_width(width);
    *proto.mutable_surface() = m_surface.to_proto();

    if (is_trimmed())
        *proto.mutable_outer_loop() = m_outer_loop.to_proto();

    for (const NurbsCurve& inner : m_inner_loops)
        *proto.add_inner_loops() = inner.to_proto();

    *proto.mutable_surfacecolor() = surfacecolor.to_proto();

    return proto;
}

NurbsSurfaceTrimmed NurbsSurfaceTrimmed::from_proto(const session_proto::NurbsSurfaceTrimmed& proto) {

    NurbsSurfaceTrimmed ts;

    if (!proto.guid().empty())
        ts.guid() = proto.guid();

    ts.name = proto.name();
    ts.width = proto.width();

    if (proto.has_surface())
        ts.m_surface = NurbsSurface::from_proto(proto.surface());

    if (proto.has_outer_loop())
        ts.m_outer_loop = NurbsCurve::from_proto(proto.outer_loop());

    for (const session_proto::NurbsCurve& loop : proto.inner_loops())
        ts.m_inner_loops.push_back(NurbsCurve::from_proto(loop));

    ts.surfacecolor = Color::from_proto(proto.surfacecolor());

    return ts;
}

std::string NurbsSurfaceTrimmed::pb_dumps() const {
    return to_proto().SerializeAsString();
}

NurbsSurfaceTrimmed NurbsSurfaceTrimmed::pb_loads(const std::string& data) {

    session_proto::NurbsSurfaceTrimmed proto;

    if (!proto.ParseFromString(data))
        throw std::runtime_error("Failed to parse NurbsSurfaceTrimmed protobuf data");

    return from_proto(proto);
}

void NurbsSurfaceTrimmed::pb_dump(const std::string& filename) const {

    const std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

NurbsSurfaceTrimmed NurbsSurfaceTrimmed::pb_load(const std::string& filename) {

    std::ifstream file(filename, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    return pb_loads(data);
}

// ═══════════════════════════════════════════════════════════════════════════
// String
// ═══════════════════════════════════════════════════════════════════════════
std::string NurbsSurfaceTrimmed::str() const {

    return fmt::format(
        "NurbsSurfaceTrimmed(name={}, trimmed={}, holes={})",
        name,
        is_trimmed() ? "true" : "false",
        inner_loop_count()
    );
}

std::string NurbsSurfaceTrimmed::repr() const {

    return fmt::format(
        "NurbsSurfaceTrimmed(\n  name={},\n  trimmed={},\n  holes={},\n  surface={}\n)",
        name,
        is_trimmed() ? "true" : "false",
        inner_loop_count(),
        m_surface.str()
    );
}

std::ostream& operator<<(std::ostream& os, const NurbsSurfaceTrimmed& ts) {

    os << ts.str();

    return os;
}

// ═══════════════════════════════════════════════════════════════════════════
// Private helpers
// ═══════════════════════════════════════════════════════════════════════════
void NurbsSurfaceTrimmed::deep_copy_from(const NurbsSurfaceTrimmed& src) {

    _guid.clear();
    name = src.name;
    width = src.width;
    surfacecolor = src.surfacecolor;
    m_surface = src.m_surface;
    m_outer_loop = src.m_outer_loop;
    m_inner_loops = src.m_inner_loops;
}

double NurbsSurfaceTrimmed::bbox_diagonal() const {

    double bmin[3] = {1e30, 1e30, 1e30}, bmax[3] = {-1e30, -1e30, -1e30};

    for (int i = 0; i < m_surface.cv_count(0); ++i)
        for (int j = 0; j < m_surface.cv_count(1); ++j) {
            Point p = m_surface.get_cv(i, j);

            for (int k = 0; k < 3; ++k) {
                if (p[k] < bmin[k])
                    bmin[k] = p[k];

                if (p[k] > bmax[k])
                    bmax[k] = p[k];
            }
        }

    double bbox_diag = std::sqrt(
        (bmax[0] - bmin[0]) * (bmax[0] - bmin[0]) + (bmax[1] - bmin[1]) * (bmax[1] - bmin[1]) +
        (bmax[2] - bmin[2]) * (bmax[2] - bmin[2])
    );

    return bbox_diag < 1e-12 ? 1.0 : bbox_diag;
}

std::vector<Point> NurbsSurfaceTrimmed::discretize_loop(const NurbsCurve& crv, double deflection) const {

    const std::vector<Point> raw = loop_points(crv);

    if (raw.size() < 2)
        return raw;

    std::vector<Point> out;
    out.reserve(raw.size() * 2);

    for (size_t i = 0; i < raw.size(); ++i)
        subdivide_edge(m_surface, raw[i], raw[(i + 1) % raw.size()], deflection, out);

    return out;
}

Mesh NurbsSurfaceTrimmed::triangulate(const TrimLoops& loops, double max_angle_deg, double chord_factor) const {

    if (loops.uv.empty() || loops.uv[0].size() < 3)
        return m_surface.mesh();

    const double bbox_diag = bbox_diagonal();
    const double deflection = bbox_diag * chord_factor;
    const double cos_max_angle = std::cos(std::min(std::max(max_angle_deg, 0.1), 179.0) * Tolerance::PI / 180.0);
    const std::array<std::vector<double>, 2> crease_knots = find_crease_knots(m_surface);
    const std::array<double, 4> bounds = loop_bounds(loops.uv[0]);

    Delaunay2D dt(bounds[0], bounds[1], bounds[2], bounds[3]);
    std::map<int, std::tuple<size_t, size_t, double>> boundary_intervals;
    const std::vector<std::vector<int>> loop_vids = insert_loops(dt, loops.uv, crease_knots, boundary_intervals);
    insert_crease_lines(dt, loops.uv, crease_knots);

    for (const Point& p : loops.interior_uv)
        if (inside_loops(p[0], p[1], loops.uv))
            dt.insert(p[0], p[1]);

    refine(dt, m_surface, loops.uv, crease_knots, deflection, cos_max_angle);
    trim_outside(dt, loops.uv);
    const std::vector<std::array<int, 3>> tris = dt.get_triangles();

    if (tris.empty() || crosses_crease(tris, dt, crease_knots))
        return Mesh();

    Mesh result;
    VertexWelder welder(result, loops.xyz.empty() ? bbox_diag * 1e-5 : 0.0, bbox_diag * 1e-5);
    const std::vector<size_t> vert_map = weld_vertices(welder, m_surface, dt, tris, loops, loop_vids, boundary_intervals);
    add_faces(result, tris, vert_map);
    set_vertex_normals(result, m_surface, dt, vert_map);
    tag_boundary(result, loop_vids, boundary_intervals, vert_map);
    RemeshNurbsSurfaceGrid::split_crease_normals(m_surface, result);

    return result;
}

} // namespace session_cpp
