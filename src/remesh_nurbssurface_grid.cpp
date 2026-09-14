#include "remesh_nurbssurface_grid.h"
#include "tolerance.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace session_cpp {
namespace {

constexpr int MAX_SUBS = 24;

// ═══════════════════════════════════════════════════════════════════════════
// Sampling
// ═══════════════════════════════════════════════════════════════════════════

/// Euclidean length without the zero gate of magnitude()
double norm(const Vector& v) {
    return std::sqrt(v.magnitude_squared());
}

/// Surface point at t along dir with the other parameter fixed
Point point_along(const NurbsSurface& s, int dir, double t, double fixed) {
    return dir == 0 ? s.point_at(t, fixed) : s.point_at(fixed, t);
}

/// Surface normal at t along dir with the other parameter fixed
Vector normal_along(const NurbsSurface& s, int dir, double t, double fixed) {
    return dir == 0 ? s.normal_at(t, fixed) : s.normal_at(fixed, t);
}

/// Sv x Su unnormalized, zero when the surface cannot be evaluated; normal_at would give a +Z sentinel at a pole
Vector raw_normal(const NurbsSurface& s, double u, double v) {
    const std::vector<Vector> derivatives = s.evaluate(u, v, 1);
    if (derivatives.size() < 3) return Vector(0.0, 0.0, 0.0);
    return derivatives[2].cross(derivatives[1]);
}

/// Diagonal of the control point bounding box
double bbox_diagonal(const NurbsSurface& s) {
    Point lo(1e30, 1e30, 1e30);
    Point hi(-1e30, -1e30, -1e30);
    for (int i = 0; i < s.cv_count(0); ++i)
        for (int j = 0; j < s.cv_count(1); ++j) {
            const Point p = s.get_cv(i, j);
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }
        }
    return norm(hi - lo);
}

// ═══════════════════════════════════════════════════════════════════════════
// Subdivisions
// ═══════════════════════════════════════════════════════════════════════════

/// Largest turn of the unit normal in degrees over [t0, t1], sampled at the span midpoints of the other direction
double span_angle(const NurbsSurface& s, int dir, double t0, double t1, const std::vector<double>& osp) {
    double max_angle = 0.0;
    for (size_t si = 0; si + 1 < osp.size(); ++si) {
        const double fixed = (osp[si] + osp[si + 1]) * 0.5;
        Vector first(0.0, 0.0, 0.0);
        Vector last(0.0, 0.0, 0.0);
        bool has_first = false;
        for (int k = 0; k <= 4; ++k) {
            const Vector n = normal_along(s, dir, t0 + k * (t1 - t0) / 4.0, fixed);
            const double length = norm(n);
            if (length < 1e-10) continue;
            const Vector unit = n / length;
            if (!has_first) first = unit;
            has_first = true;
            last = unit;
        }
        if (!has_first) continue;
        const double dot = std::max(-1.0, std::min(1.0, first.dot(last)));
        max_angle = std::max(max_angle, std::acos(dot) * 180.0 / Tolerance::PI);
    }
    return max_angle;
}

/// Largest height of [t0, t1] over its chord, at up to four positions across the other direction
double span_deviation(const NurbsSurface& s, int dir, double t0, double t1, const std::vector<double>& osp) {
    double max_dev = 0.0;
    const int nc = std::min((int)osp.size() - 1, 3);
    for (int ci = 0; ci <= nc; ++ci) {
        const double fixed = osp.front() + ci * (osp.back() - osp.front()) / std::max(nc, 1);
        const Point p0 = point_along(s, dir, t0, fixed);
        const Point p1 = point_along(s, dir, t1, fixed);
        for (int k = 1; k <= 3; ++k) {
            const double frac = k / 4.0;
            const Point pm = point_along(s, dir, t0 + frac * (t1 - t0), fixed);
            max_dev = std::max(max_dev, norm(pm - (p0 + (p1 - p0) * frac)));
        }
    }
    return max_dev;
}

/// Subdivisions per span along dir: the normal turn against max_angle_deg, the chord height against chord_tol, at least two on a curved span
std::vector<int> span_subs(const NurbsSurface& s, int dir, const std::vector<double>& sp, const std::vector<double>& osp, double max_angle_deg, double chord_tol) {
    const int degree = s.degree(dir);
    std::vector<int> subs(sp.size() - 1, 1);
    for (size_t i = 0; i + 1 < sp.size(); ++i) {
        if (degree > 1) {
            const double angle = span_angle(s, dir, sp[i], sp[i + 1], osp);
            subs[i] = std::clamp((int)std::ceil(angle / max_angle_deg), 1, MAX_SUBS);
        }
        const double dev = span_deviation(s, dir, sp[i], sp[i + 1], osp);
        if (dev > chord_tol) subs[i] = std::max(subs[i], std::clamp((int)std::ceil(std::sqrt(dev / chord_tol)), 2, MAX_SUBS));
        if (degree > 1) subs[i] = std::max(subs[i], 2);
    }
    return subs;
}

/// Length of the iso-curve at fixed along dir as a polyline of n steps
double isocurve_length(const NurbsSurface& s, int dir, const std::vector<double>& sp, double fixed, int n) {
    double length = 0.0;
    Point prev = point_along(s, dir, sp.front(), fixed);
    for (int i = 1; i <= n; ++i) {
        const Point next = point_along(s, dir, sp.front() + i * (sp.back() - sp.front()) / n, fixed);
        length += norm(next - prev);
        prev = next;
    }
    return length;
}

/// Scale up the curved direction whose spacing is more than twice the other's
void balance_subs(const NurbsSurface& s, const std::vector<double>& usp, const std::vector<double>& vsp, std::vector<int>& u_subs, std::vector<int>& v_subs) {
    int total_u = 1;
    int total_v = 1;
    for (int sub : u_subs) total_u += sub;
    for (int sub : v_subs) total_v += sub;
    const double u_len = isocurve_length(s, 0, usp, (vsp.front() + vsp.back()) * 0.5, std::max(total_u, 10));
    const double v_len = isocurve_length(s, 1, vsp, (usp.front() + usp.back()) * 0.5, std::max(total_v, 10));
    if (u_len <= 1e-14 || v_len <= 1e-14) return;
    const double ratio = (u_len / total_u) / (v_len / total_v);
    if (ratio > 2.0 && s.degree(0) > 1) {
        const double scale = std::sqrt(ratio);
        for (int& sub : u_subs) sub = std::min(MAX_SUBS, (int)std::ceil(sub * scale));
    } else if (ratio < 0.5 && s.degree(1) > 1) {
        const double scale = std::sqrt(1.0 / ratio);
        for (int& sub : v_subs) sub = std::min(MAX_SUBS, (int)std::ceil(sub * scale));
    }
}

/// Subdivisions both directions of a bilinear surface need for its twist, 1 when every span centre lies within twist_tol of its diagonal midpoint
int twist_subs(const NurbsSurface& s, const std::vector<double>& usp, const std::vector<double>& vsp, double twist_tol) {
    double max_twist = 0.0;
    for (size_t i = 0; i + 1 < usp.size(); ++i)
        for (size_t j = 0; j + 1 < vsp.size(); ++j) {
            const Point pm = s.point_at((usp[i] + usp[i + 1]) * 0.5, (vsp[j] + vsp[j + 1]) * 0.5);
            const Point p00 = s.point_at(usp[i], vsp[j]);
            const Point p11 = s.point_at(usp[i + 1], vsp[j + 1]);
            max_twist = std::max(max_twist, norm(pm - Point::sum(p00, p11) * 0.5));
        }
    if (max_twist <= twist_tol) return 1;
    return std::clamp((int)std::ceil(2.0 * std::sqrt(max_twist / twist_tol)), 4, MAX_SUBS);
}

/// One more subdivision on the largest span when the total is even, so a closed direction triangulates seamlessly
void make_odd(std::vector<int>& subs) {
    int total = 0;
    for (int sub : subs) total += sub;
    if (total % 2 == 0) *std::max_element(subs.begin(), subs.end()) += 1;
}

// ═══════════════════════════════════════════════════════════════════════════
// Parameters
// ═══════════════════════════════════════════════════════════════════════════

/// n parameters spaced evenly by arc length along the iso-curve at fixed
std::vector<double> arclen_params(const NurbsSurface& s, int dir, int n, const std::vector<double>& sp, double fixed) {
    const int nsample = std::max(n * 20, 200);
    std::vector<double> st(nsample + 1);
    std::vector<double> sl(nsample + 1, 0.0);
    Point prev = point_along(s, dir, sp.front(), fixed);
    for (int k = 0; k <= nsample; ++k) {
        st[k] = sp.front() + k * (sp.back() - sp.front()) / nsample;
        if (k == 0) continue;
        const Point next = point_along(s, dir, st[k], fixed);
        sl[k] = sl[k - 1] + norm(next - prev);
        prev = next;
    }
    std::vector<double> params;
    params.push_back(sp.front());
    int j = 0;
    for (int i = 1; i < n - 1; ++i) {
        const double target = sl[nsample] * i / (n - 1);
        while (j < nsample && sl[j] < target) ++j;
        const int a = j > 0 ? j - 1 : 0;
        const double frac = sl[j] > sl[a] ? (target - sl[a]) / (sl[j] - sl[a]) : 0.0;
        params.push_back(st[a] + frac * (st[j] - st[a]));
    }
    params.push_back(sp.back());
    return params;
}

/// Every span split into its subdivisions, ending on the last span boundary
std::vector<double> span_params(const std::vector<double>& sp, const std::vector<int>& subs) {
    std::vector<double> params;
    for (size_t i = 0; i + 1 < sp.size(); ++i)
        for (int sub = 0; sub < subs[i]; ++sub)
            params.push_back(sp[i] + sub * (sp[i + 1] - sp[i]) / subs[i]);
    params.push_back(sp.back());
    return params;
}

/// Closed direction: drop the duplicate end and fill a wrap gap wider than 1.5 times the largest step
void fix_closed_gap(std::vector<double>& params, double domain_end) {
    if (params.size() < 3) return;
    params.pop_back();
    const double wrap_gap = domain_end - params.back();
    double max_gap = 0.0;
    for (size_t i = 1; i < params.size(); ++i) max_gap = std::max(max_gap, params[i] - params[i - 1]);
    if (max_gap <= 0.0 || wrap_gap <= max_gap * 1.5) return;
    const int extra = (int)std::ceil(wrap_gap / max_gap) - 1;
    const double step = wrap_gap / (extra + 1);
    for (int e = 1; e <= extra; ++e) params.push_back(params.back() + step);
}

// ═══════════════════════════════════════════════════════════════════════════
// Vertices and faces
// ═══════════════════════════════════════════════════════════════════════════

/// Vertex at S(u, v) tagged with its parameters
size_t add_vertex_uv(const NurbsSurface& s, Mesh& mesh, double u, double v) {
    const size_t key = mesh.add_vertex(s.point_at(u, v));
    mesh.vertex[key].attributes["u"] = u;
    mesh.vertex[key].attributes["v"] = v;
    return key;
}

/// Grid vertices row by row over us and the rows j_start..j_end of vs
std::vector<size_t> add_grid(const NurbsSurface& s, Mesh& mesh, const std::vector<double>& us, const std::vector<double>& vs, int j_start, int j_end) {
    std::vector<size_t> grid;
    for (double u : us)
        for (int j = j_start; j < j_end; ++j)
            grid.push_back(add_vertex_uv(s, mesh, u, vs[j]));
    return grid;
}

/// Fans from the south pole, checkerboard-split quads, fans to the north pole
void add_faces(Mesh& mesh, const std::vector<size_t>& grid, int nu, bool closed_u, bool wrap_v, std::optional<size_t> south, std::optional<size_t> north) {
    const int nv = (int)grid.size() / nu;
    const int nu_faces = closed_u ? nu : nu - 1;
    const int nv_faces = wrap_v ? nv : nv - 1;
    if (south)
        for (int i = 0; i < nu_faces; ++i) mesh.add_face({*south, grid[((i + 1) % nu) * nv], grid[i * nv]});
    for (int i = 0; i < nu_faces; ++i)
        for (int j = 0; j < nv_faces; ++j) {
            const int i1 = (i + 1) % nu;
            const int j1 = (j + 1) % nv;
            const size_t v00 = grid[i * nv + j];
            const size_t v10 = grid[i1 * nv + j];
            const size_t v01 = grid[i * nv + j1];
            const size_t v11 = grid[i1 * nv + j1];
            if ((i + j) % 2 == 0) {
                mesh.add_face({v00, v10, v11});
                mesh.add_face({v00, v11, v01});
            } else {
                mesh.add_face({v00, v10, v01});
                mesh.add_face({v10, v11, v01});
            }
        }
    if (north)
        for (int i = 0; i < nu_faces; ++i) mesh.add_face({grid[i * nv + nv - 1], grid[((i + 1) % nu) * nv + nv - 1], *north});
}

// ═══════════════════════════════════════════════════════════════════════════
// Normals
// ═══════════════════════════════════════════════════════════════════════════

/// Sum of the unnormalized face normals around each vertex key, faces taken in key order
std::vector<Vector> fan_normals(const Mesh& mesh) {
    std::vector<Vector> sums(mesh.vertex.size(), Vector(0.0, 0.0, 0.0));
    for (const auto& [key, vertices] : mesh.face) {
        if (vertices.size() < 3) continue;
        const Point p0 = mesh.vertex.at(vertices[0]).position();
        const Point p1 = mesh.vertex.at(vertices[1]).position();
        const Point p2 = mesh.vertex.at(vertices[2]).position();
        const Vector n = (p1 - p0).cross(p2 - p0);
        for (size_t vertex : vertices) sums[vertex] += n;
    }
    return sums;
}

/// Unit surface normal on the side of the fan normal; the fan normal at the poles and where the surface normal vanishes, +Z when the fan vanishes too
void set_normals(const NurbsSurface& s, Mesh& mesh, std::optional<size_t> south, std::optional<size_t> north) {
    const std::vector<Vector> sums = fan_normals(mesh);
    for (auto& [key, vd] : mesh.vertex) {
        Vector n(0.0, 0.0, 1.0);
        const double fan_length = norm(sums[key]);
        if (std::isfinite(fan_length) && fan_length > 0.0) n = sums[key] / fan_length;
        if (key != south && key != north) {
            const Vector raw = raw_normal(s, vd.attributes.at("u"), vd.attributes.at("v"));
            const double length = norm(raw);
            if (std::isfinite(length) && length > 0.0) n = raw.dot(n) < 0.0 ? -raw / length : raw / length;
        }
        vd.set_normal(n[0], n[1], n[2]);
    }
}

/// Bit per direction where (u, v) sits on an internal knot of full multiplicity whose one-sided normals disagree
unsigned crease_flags(const NurbsSurface& s, double u, double v) {
    const double uv[2] = {u, v};
    unsigned flags = 0;
    for (int dir = 0; dir < 2; ++dir) {
        const auto [start, end] = s.domain(dir);
        const double value = uv[dir];
        if (value <= start || value >= end) continue;
        if (std::count(s.m_nurbsknot[dir].begin(), s.m_nurbsknot[dir].end(), value) < s.degree(dir)) continue;
        double lo[2] = {u, v};
        double hi[2] = {u, v};
        lo[dir] = std::nextafter(value, -std::numeric_limits<double>::infinity());
        hi[dir] = std::nextafter(value, std::numeric_limits<double>::infinity());
        const Vector a = s.normal_at(lo[0], lo[1]);
        const Vector b = s.normal_at(hi[0], hi[1]);
        const double length = std::sqrt(a.magnitude_squared() * b.magnitude_squared());
        if (length == 0.0) continue;
        const double dot = a.dot(b) / length;
        if (std::isfinite(dot) && dot < 1.0 - 64.0 * std::numeric_limits<double>::epsilon()) flags |= 1u << dir;
    }
    return flags;
}

/// Nudge uv one ulp toward center in each flagged direction; bit per direction nudged upward
unsigned crease_side(const double center[2], double uv[2], unsigned flags) {
    unsigned side = 0;
    for (int dir = 0; dir < 2; ++dir) {
        if (!(flags & (1u << dir))) continue;
        const bool high = center[dir] > uv[dir];
        if (high) side |= 1u << dir;
        uv[dir] = std::nextafter(uv[dir], high ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity());
    }
    return side;
}

/// Vertex carrying a corner: the original the first time its key is met, then one copy per (key, side)
size_t crease_target(Mesh& mesh, std::map<std::pair<size_t, unsigned>, size_t>& copies, std::set<size_t>& used, size_t key, unsigned side) {
    const std::pair<size_t, unsigned> identity(key, side);
    if (copies.count(identity)) return copies[identity];
    if (used.insert(key).second) return copies[identity] = key;
    const size_t target = mesh.add_vertex(mesh.vertex[key].position());
    mesh.vertex[target] = mesh.vertex[key];
    return copies[identity] = target;
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════════
// RemeshNurbsSurfaceGrid
// ═══════════════════════════════════════════════════════════════════════════

Mesh RemeshNurbsSurfaceGrid::from_u_v(const NurbsSurface& s, int max_u, int max_v) {
    return from_u_v_q(s, max_u, max_v, 20.0, 0.005);
}

Mesh RemeshNurbsSurfaceGrid::from_u_v_q(const NurbsSurface& s, int max_u, int max_v, double max_angle_deg, double chord_factor) {
    const std::vector<double> usp = s.get_span_vector(0);
    const std::vector<double> vsp = s.get_span_vector(1);
    const double bbox_diag = bbox_diagonal(s);
    const double chord_tol = bbox_diag * chord_factor;
    std::vector<int> u_subs = span_subs(s, 0, usp, vsp, max_angle_deg, chord_tol);
    std::vector<int> v_subs = span_subs(s, 1, vsp, usp, max_angle_deg, chord_tol);
    balance_subs(s, usp, vsp, u_subs, v_subs);
    const bool sing_v0 = s.is_singular(0);
    const bool sing_v1 = s.is_singular(2);
    if (s.degree(0) == 1 && s.degree(1) == 1 && !sing_v0 && !sing_v1) {
        const int twist = twist_subs(s, usp, vsp, bbox_diag > 0.0 ? chord_tol : 1e-6);
        for (int& sub : u_subs) sub = std::max(sub, twist);
        for (int& sub : v_subs) sub = std::max(sub, twist);
    }
    const bool closed_u = s.is_closed(0);
    const bool closed_v = s.is_closed(1);
    if (closed_u && max_u == 0) make_odd(u_subs);
    if (closed_v && max_v == 0) make_odd(v_subs);
    const double u_mid = (usp.front() + usp.back()) * 0.5;
    const double v_mid = (vsp.front() + vsp.back()) * 0.5;
    std::vector<double> us = max_u > 0 ? arclen_params(s, 0, std::max(max_u, 2), usp, v_mid) : span_params(usp, u_subs);
    std::vector<double> vs = max_v > 0 ? arclen_params(s, 1, std::max(max_v, 2), vsp, u_mid) : span_params(vsp, v_subs);
    if (closed_u) fix_closed_gap(us, usp.back());
    if (closed_v) fix_closed_gap(vs, vsp.back());
    const int nv = (int)vs.size();
    Mesh mesh;
    std::optional<size_t> south;
    std::optional<size_t> north;
    if (sing_v0) south = add_vertex_uv(s, mesh, us[0], vs[0]);
    if (sing_v1) north = add_vertex_uv(s, mesh, us[0], vs[nv - 1]);
    const std::vector<size_t> grid = add_grid(s, mesh, us, vs, sing_v0 ? 1 : 0, sing_v1 ? nv - 1 : nv);
    add_faces(mesh, grid, (int)us.size(), closed_u, closed_v && !sing_v0 && !sing_v1, south, north);
    set_normals(s, mesh, south, north);
    split_crease_normals(s, mesh);
    return mesh;
}

void RemeshNurbsSurfaceGrid::split_crease_normals(const NurbsSurface& s, Mesh& mesh) {
    std::map<size_t, unsigned> candidates;
    for (const auto& [key, vd] : mesh.vertex) {
        if (!vd.attributes.count("u") || !vd.attributes.count("v")) continue;
        const unsigned flags = crease_flags(s, vd.attributes.at("u"), vd.attributes.at("v"));
        if (flags) candidates[key] = flags;
    }
    if (candidates.empty()) return;
    std::map<std::pair<size_t, unsigned>, size_t> copies;
    std::set<size_t> used;
    for (auto& [face_key, vertices] : mesh.face) {
        double center[2] = {0.0, 0.0};
        for (size_t key : vertices) {
            center[0] += mesh.vertex[key].attributes.at("u");
            center[1] += mesh.vertex[key].attributes.at("v");
        }
        center[0] /= vertices.size();
        center[1] /= vertices.size();
        const std::optional<Vector> face_normal = mesh.face_normal(face_key);
        for (size_t& key : vertices) {
            if (!candidates.count(key)) continue;
            double uv[2] = {mesh.vertex[key].attributes.at("u"), mesh.vertex[key].attributes.at("v")};
            const unsigned side = crease_side(center, uv, candidates[key]);
            const size_t target = crease_target(mesh, copies, used, key, side);
            const Vector n = s.normal_at(uv[0], uv[1]);
            const double length = norm(n);
            if (std::isfinite(length) && length > 0.0) {
                const double sign = face_normal && n.dot(*face_normal) < 0.0 ? -1.0 : 1.0;
                mesh.vertex[target].set_normal(sign * n[0] / length, sign * n[1] / length, sign * n[2] / length);
            }
            key = target;
        }
    }
    mesh.rebuild_halfedges();
}

} // namespace session_cpp
