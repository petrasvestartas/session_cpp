#include "trimesh_delaunay.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <array>

namespace session_cpp {

using Pos3 = std::array<double, 3>;

///////////////////////////////////////////////////////////////////////////////////////////
// Delaunay2D Implementation
///////////////////////////////////////////////////////////////////////////////////////////

double Delaunay2D::in_circumcircle(double ax, double ay, double bx, double by,
                                    double cx, double cy, double dx, double dy) {
    double adx = ax - dx, ady = ay - dy;
    double bdx = bx - dx, bdy = by - dy;
    double cdx = cx - dx, cdy = cy - dy;
    return (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy)
         + (bdx * bdx + bdy * bdy) * (cdx * ady - adx * cdy)
         + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);
}

double Delaunay2D::orient2d(double ax, double ay, double bx, double by,
                             double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

void Delaunay2D::circumcenter(double ax, double ay, double bx, double by,
                               double cx, double cy, double& ux, double& uy) {
    double D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if (std::abs(D) < 1e-30) {
        ux = (ax + bx + cx) / 3.0;
        uy = (ay + by + cy) / 3.0;
        return;
    }
    double a2 = ax * ax + ay * ay;
    double b2 = bx * bx + by * by;
    double c2 = cx * cx + cy * cy;
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / D;
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / D;
}

void Delaunay2D::register_triangle_edges(int ti) {
    const Triangle& t = triangles[ti];
    for (int k = 0; k < 3; ++k) {
        int a = t.v[(k + 1) % 3];
        int b = t.v[(k + 2) % 3];
        auto key = edge_key(a, b);
        auto* val = edge_map.find(key);
        if (val) {
            auto [oti, ok] = *val;
            triangles[ti].adj[k] = oti;
            triangles[oti].adj[ok] = ti;
            edge_map.erase(key);
        } else {
            edge_map[key] = {ti, k};
        }
    }
}

void Delaunay2D::unregister_triangle_edges(int ti) {
    const Triangle& t = triangles[ti];
    for (int k = 0; k < 3; ++k) {
        int a = t.v[(k + 1) % 3];
        int b = t.v[(k + 2) % 3];
        auto key = edge_key(a, b);
        int adj_ti = t.adj[k];
        if (adj_ti >= 0 && triangles[adj_ti].alive) {
            Triangle& adj = triangles[adj_ti];
            for (int kk = 0; kk < 3; ++kk) {
                if (adj.adj[kk] == ti) {
                    adj.adj[kk] = -1;
                    edge_map[edge_key(adj.v[(kk + 1) % 3], adj.v[(kk + 2) % 3])] = {adj_ti, kk};
                    break;
                }
            }
        } else {
            auto* val = edge_map.find(key);
            if (val && val->first == ti)
                edge_map.erase(key);
        }
    }
}

Delaunay2D::Delaunay2D(double xmin, double ymin, double xmax, double ymax) {
    double dx = xmax - xmin;
    double dy = ymax - ymin;
    double d = std::max(dx, dy);
    double cx = (xmin + xmax) * 0.5;
    double cy = (ymin + ymax) * 0.5;
    double scale = 20.0;

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
    t.v[0] = 0; t.v[1] = 1; t.v[2] = 2;
    t.adj[0] = -1; t.adj[1] = -1; t.adj[2] = -1;
    t.alive = true;
    triangles.push_back(t);
    register_triangle_edges(0);
}

int Delaunay2D::locate(double x, double y, int start_tri) const {
    if (start_tri < 0 || start_tri >= (int)triangles.size() || !triangles[start_tri].alive) {
        start_tri = (int)triangles.size() - 1;
        while (start_tri >= 0 && !triangles[start_tri].alive) --start_tri;
        if (start_tri < 0) return -1;
    }
    int cur = start_tri;
    int max_iter = (int)triangles.size();
    for (int iter = 0; iter < max_iter; ++iter) {
        const Triangle& tri = triangles[cur];
        bool moved = false;
        for (int k = 0; k < 3; ++k) {
            int a = tri.v[k], b = tri.v[(k + 1) % 3];
            double o = orient2d(vertices[a].x, vertices[a].y,
                                vertices[b].x, vertices[b].y, x, y);
            if (o < 0) {
                int opp = (k + 2) % 3;
                if (tri.adj[opp] >= 0 && triangles[tri.adj[opp]].alive) {
                    cur = tri.adj[opp];
                    moved = true;
                    break;
                }
            }
        }
        if (!moved) return cur;
    }
    return cur;
}

int Delaunay2D::insert(double x, double y) {
    int start = locate(x, y, last_found_);

    if (start >= 0 && triangles[start].alive) {
        for (int k = 0; k < 3; ++k) {
            int vi2 = triangles[start].v[k];
            double ddx = vertices[vi2].x - x;
            double ddy = vertices[vi2].y - y;
            if (ddx * ddx + ddy * ddy < 1e-12) return vi2;
        }
    }

    int vi = (int)vertices.size();
    vertices.push_back({x, y});

    ++visit_epoch_;
    if ((int)visit_stamp_.size() < (int)triangles.size() + 64)
        visit_stamp_.resize(triangles.size() + 64, 0);

    bad_.clear();
    int bfs_front = 0;
    bad_.push_back(start);
    visit_stamp_[start] = visit_epoch_;

    while (bfs_front < (int)bad_.size()) {
        int ti = bad_[bfs_front++];
        const Triangle& tri = triangles[ti];
        if (!tri.alive) { bad_[bfs_front - 1] = -1; continue; }

        const Vertex2D& a = vertices[tri.v[0]];
        const Vertex2D& b = vertices[tri.v[1]];
        const Vertex2D& c = vertices[tri.v[2]];
        double o = orient2d(a.x, a.y, b.x, b.y, c.x, c.y);
        double ic;
        if (o > 0)
            ic = in_circumcircle(a.x, a.y, b.x, b.y, c.x, c.y, x, y);
        else
            ic = in_circumcircle(a.x, a.y, c.x, c.y, b.x, b.y, x, y);

        if (ic > 0) {
            for (int k = 0; k < 3; ++k) {
                int nb = tri.adj[k];
                if (nb >= 0 && nb < (int)visit_stamp_.size() && visit_stamp_[nb] != visit_epoch_) {
                    visit_stamp_[nb] = visit_epoch_;
                    bad_.push_back(nb);
                }
            }
        } else {
            bad_[bfs_front - 1] = -1;
        }
    }

    int write = 0;
    for (int i = 0; i < (int)bad_.size(); ++i)
        if (bad_[i] >= 0) bad_[write++] = bad_[i];
    bad_.resize(write);

    if (bad_.empty()) {
        vertices.pop_back();
        return -1;
    }

    polygon_.clear();
    for (int ti : bad_) {
        const Triangle& tri = triangles[ti];
        for (int k = 0; k < 3; ++k) {
            int nb = tri.adj[k];
            bool nb_bad = false;
            if (nb >= 0) {
                for (int bi : bad_) {
                    if (bi == nb) { nb_bad = true; break; }
                }
            }
            if (!nb_bad) {
                polygon_.push_back({tri.v[(k + 1) % 3], tri.v[(k + 2) % 3]});
            }
        }
    }

    for (int ti : bad_) {
        unregister_triangle_edges(ti);
        triangles[ti].alive = false;
    }

    for (const auto& edge : polygon_) {
        double o = orient2d(vertices[vi].x, vertices[vi].y,
                            vertices[edge.e0].x, vertices[edge.e0].y,
                            vertices[edge.e1].x, vertices[edge.e1].y);
        if (std::abs(o) < 1e-20) continue;
        int new_ti = (int)triangles.size();
        Triangle nt;
        nt.v[0] = vi;
        if (o > 0) { nt.v[1] = edge.e0; nt.v[2] = edge.e1; }
        else       { nt.v[1] = edge.e1; nt.v[2] = edge.e0; }
        nt.adj[0] = -1; nt.adj[1] = -1; nt.adj[2] = -1;
        nt.alive = true;
        triangles.push_back(nt);
        register_triangle_edges(new_ti);
    }

    last_found_ = (int)triangles.size() - 1;

    return vi;
}

void Delaunay2D::insert_constraint(int v0, int v1) {
    if (v0 == v1) return;
    for (auto& tri : triangles) {
        if (!tri.alive) continue;
        for (int k = 0; k < 3; ++k) {
            int e0 = tri.v[(k + 1) % 3];
            int e1 = tri.v[(k + 2) % 3];
            if ((e0 == v0 && e1 == v1) || (e0 == v1 && e1 == v0))
                tri.constrained[k] = true;
        }
    }
}

void Delaunay2D::cleanup() {
    for (auto& tri : triangles) {
        if (!tri.alive) continue;
        for (int k = 0; k < 3; ++k) {
            if (tri.v[k] == super_v[0] || tri.v[k] == super_v[1] || tri.v[k] == super_v[2]) {
                unregister_triangle_edges((int)(&tri - &triangles[0]));
                tri.alive = false;
                break;
            }
        }
    }
    last_found_ = 0;
    for (int i = 0; i < (int)triangles.size(); ++i) {
        if (triangles[i].alive) { last_found_ = i; break; }
    }
}

std::vector<std::array<int, 3>> Delaunay2D::get_triangles() const {
    std::vector<std::array<int, 3>> result;
    for (const auto& tri : triangles) {
        if (!tri.alive) continue;
        double o = orient2d(vertices[tri.v[0]].x, vertices[tri.v[0]].y,
                            vertices[tri.v[1]].x, vertices[tri.v[1]].y,
                            vertices[tri.v[2]].x, vertices[tri.v[2]].y);
        if (o > 0)
            result.push_back({tri.v[0], tri.v[1], tri.v[2]});
        else
            result.push_back({tri.v[0], tri.v[2], tri.v[1]});
    }
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// TrimeshDelaunay Implementation
///////////////////////////////////////////////////////////////////////////////////////////

TrimeshDelaunay::TrimeshDelaunay(const NurbsSurface& surface)
    : m_surface(surface) {}

TrimeshDelaunay& TrimeshDelaunay::set_max_angle(double degrees) {
    m_max_angle = degrees;
    return *this;
}

TrimeshDelaunay& TrimeshDelaunay::set_max_edge_length(double length) {
    m_max_edge_length = length;
    return *this;
}

TrimeshDelaunay& TrimeshDelaunay::set_min_edge_length(double length) {
    m_min_edge_length = length;
    return *this;
}

TrimeshDelaunay& TrimeshDelaunay::set_max_chord_height(double height) {
    m_max_chord_height = height;
    return *this;
}

TrimeshDelaunay& TrimeshDelaunay::set_max_iterations(int iterations) {
    m_max_iterations = iterations;
    return *this;
}

double TrimeshDelaunay::compute_bbox_diagonal() const {
    double minx = 1e30, miny = 1e30, minz = 1e30;
    double maxx = -1e30, maxy = -1e30, maxz = -1e30;
    for (int i = 0; i < m_surface.cv_count(0); ++i) {
        for (int j = 0; j < m_surface.cv_count(1); ++j) {
            Point p = m_surface.get_cv(i, j);
            if (p[0] < minx) minx = p[0];
            if (p[1] < miny) miny = p[1];
            if (p[2] < minz) minz = p[2];
            if (p[0] > maxx) maxx = p[0];
            if (p[1] > maxy) maxy = p[1];
            if (p[2] > maxz) maxz = p[2];
        }
    }
    double dx = maxx - minx;
    double dy = maxy - miny;
    double dz = maxz - minz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static bool point_in_polygon(double px, double py,
                             const std::vector<Vertex2D>& poly) {
    int n = (int)poly.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = poly[i].x, yi = poly[i].y;
        double xj = poly[j].x, yj = poly[j].y;
        if (((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
            inside = !inside;
    }
    return inside;
}

static inline double dot3(const Pos3& a, const Pos3& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

///////////////////////////////////////////////////////////////////////////////////////////
// Scoring
///////////////////////////////////////////////////////////////////////////////////////////

struct TriScore {
    double score;
    int tri_idx;
    bool operator<(const TriScore& o) const { return score < o.score; }
};

static double score_triangle(
    int ti, const Delaunay2D& dt,
    const std::vector<Pos3>& pos_cache,
    const std::vector<Pos3>& nrm_cache,
    FlatMap64<double>& edge_chord_cache,
    const NurbsSurface& surface,
    double max_edge_sq, double chord_tol_sq, double cos_angle, double min_edge_sq,
    double max_edge, double chord_tol, double angle_rad)
{
    const Triangle& tri = dt.triangles[ti];
    if (!tri.alive) return -1.0;

    for (int k = 0; k < 3; ++k)
        if (tri.v[k] == dt.super_v[0] || tri.v[k] == dt.super_v[1] || tri.v[k] == dt.super_v[2])
            return -1.0;

    const Pos3& p0 = pos_cache[tri.v[0]];
    const Pos3& p1 = pos_cache[tri.v[1]];
    const Pos3& p2 = pos_cache[tri.v[2]];

    auto dist_sq = [](const Pos3& a, const Pos3& b) {
        double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
        return dx*dx + dy*dy + dz*dz;
    };
    double e01sq = dist_sq(p0, p1);
    double e12sq = dist_sq(p1, p2);
    double e20sq = dist_sq(p2, p0);
    double max_esq = std::max({e01sq, e12sq, e20sq});

    if (max_esq < min_edge_sq) return -1.0;

    double score = 0.0;

    if (max_esq > max_edge_sq)
        score = std::max(score, std::sqrt(max_esq) / max_edge);

    const Pos3& n0 = nrm_cache[tri.v[0]];
    const Pos3& n1 = nrm_cache[tri.v[1]];
    const Pos3& n2 = nrm_cache[tri.v[2]];

    auto check_angle = [&](const Pos3& na, const Pos3& nb) {
        double d = dot3(na, nb);
        if (d < cos_angle) {
            d = std::max(-1.0, std::min(1.0, d));
            double angle = std::acos(d);
            score = std::max(score, angle / angle_rad);
        }
    };
    check_angle(n0, n1);
    check_angle(n1, n2);
    check_angle(n2, n0);

    double u0 = dt.vertices[tri.v[0]].x, v0 = dt.vertices[tri.v[0]].y;
    double u1 = dt.vertices[tri.v[1]].x, v1 = dt.vertices[tri.v[1]].y;
    double u2 = dt.vertices[tri.v[2]].x, v2 = dt.vertices[tri.v[2]].y;

    auto check_edge = [&](int va, int vb, const Pos3& pa, const Pos3& pb,
                          double ua, double va2, double ub, double vb2) {
        auto key = Delaunay2D::edge_key(va, vb);
        double* cached = edge_chord_cache.find(key);
        double h_sq;
        if (cached) {
            h_sq = *cached;
        } else {
            double um = (ua + ub) * 0.5;
            double vm = (va2 + vb2) * 0.5;
            double mx, my, mz;
            surface.point_at(um, vm, mx, my, mz);
            double ex = (pa[0]+pb[0])*0.5 - mx, ey = (pa[1]+pb[1])*0.5 - my, ez = (pa[2]+pb[2])*0.5 - mz;
            h_sq = ex*ex + ey*ey + ez*ez;
            edge_chord_cache[key] = h_sq;
        }
        if (h_sq > chord_tol_sq)
            score = std::max(score, std::sqrt(h_sq) / chord_tol);
    };
    check_edge(tri.v[0], tri.v[1], p0, p1, u0, v0, u1, v1);
    check_edge(tri.v[1], tri.v[2], p1, p2, u1, v1, u2, v2);
    check_edge(tri.v[2], tri.v[0], p2, p0, u2, v2, u0, v0);

    return score;
}

///////////////////////////////////////////////////////////////////////////////////////////
// mesh()
///////////////////////////////////////////////////////////////////////////////////////////

Mesh TrimeshDelaunay::mesh() const {
    auto [u_min, u_max] = m_surface.domain(0);
    auto [v_min, v_max] = m_surface.domain(1);

    double bbox_diag = compute_bbox_diagonal();
    double max_edge = m_max_edge_length > 0 ? m_max_edge_length : bbox_diag / 10.0;
    double min_edge = m_min_edge_length > 0 ? m_min_edge_length : bbox_diag / 1000.0;
    double chord_tol = m_max_chord_height > 0 ? m_max_chord_height : bbox_diag / 500.0;
    double angle_rad = m_max_angle * Tolerance::PI / 180.0;

    bool trimmed = m_surface.get_outer_loop().is_valid();
    std::vector<Vertex2D> trim_poly;
    std::vector<std::vector<Vertex2D>> hole_polys;

    Delaunay2D dt(u_min, v_min, u_max, v_max);

    std::vector<Pos3> pos_cache;
    std::vector<Pos3> nrm_cache;

    auto cache_vertex = [&](int vi) {
        while ((int)pos_cache.size() <= vi) {
            pos_cache.push_back({0, 0, 0});
            nrm_cache.push_back({0, 0, 1});
        }
        bool is_super = (vi == dt.super_v[0] || vi == dt.super_v[1] || vi == dt.super_v[2]);
        if (!is_super) {
            double u = dt.vertices[vi].x;
            double v = dt.vertices[vi].y;
            m_surface.point_and_normal_at(u, v,
                pos_cache[vi][0], pos_cache[vi][1], pos_cache[vi][2],
                nrm_cache[vi][0], nrm_cache[vi][1], nrm_cache[vi][2]);
        }
    };

    for (int k = 0; k < 3; ++k) cache_vertex(dt.super_v[k]);

    auto insert_and_cache = [&](double u, double v) -> int {
        int vi = dt.insert(u, v);
        if (vi >= 0) cache_vertex(vi);
        return vi;
    };

    if (trimmed) {
        NurbsCurve loop = m_surface.get_outer_loop();
        for (int i = 0; i < loop.cv_count(); ++i) {
            Point cv = loop.get_cv(i);
            trim_poly.push_back({cv[0], cv[1]});
        }

        std::vector<int> trim_verts;
        for (const auto& v : trim_poly)
            trim_verts.push_back(insert_and_cache(v.x, v.y));

        for (int hi = 0; hi < m_surface.inner_loop_count(); ++hi) {
            NurbsCurve hole_loop = m_surface.get_inner_loop(hi);
            std::vector<Vertex2D> hole_poly;
            std::vector<int> hole_verts;
            for (int i = 0; i < hole_loop.cv_count(); ++i) {
                Point cv = hole_loop.get_cv(i);
                hole_poly.push_back({cv[0], cv[1]});
                hole_verts.push_back(insert_and_cache(cv[0], cv[1]));
            }
            for (size_t i = 0; i + 1 < hole_verts.size(); ++i) {
                if (hole_verts[i] >= 0 && hole_verts[i + 1] >= 0)
                    dt.insert_constraint(hole_verts[i], hole_verts[i + 1]);
            }
            hole_polys.push_back(std::move(hole_poly));
        }

        if (!m_surface.is_planar()) {
            int grid_n = 8;
            double du = (u_max - u_min) / (grid_n + 1);
            double dv = (v_max - v_min) / (grid_n + 1);
            for (int i = 1; i <= grid_n; ++i) {
                for (int j = 1; j <= grid_n; ++j) {
                    double gu = u_min + i * du;
                    double gv = v_min + j * dv;
                    if (!point_in_polygon(gu, gv, trim_poly)) continue;
                    bool in_hole = false;
                    for (const auto& hp : hole_polys) {
                        if (point_in_polygon(gu, gv, hp)) { in_hole = true; break; }
                    }
                    if (!in_hole) insert_and_cache(gu, gv);
                }
            }
        }

        for (size_t i = 0; i + 1 < trim_verts.size(); ++i) {
            if (trim_verts[i] >= 0 && trim_verts[i + 1] >= 0)
                dt.insert_constraint(trim_verts[i], trim_verts[i + 1]);
        }

        dt.cleanup();

        for (auto& tri : dt.triangles) {
            if (!tri.alive) continue;
            double cu = (dt.vertices[tri.v[0]].x + dt.vertices[tri.v[1]].x + dt.vertices[tri.v[2]].x) / 3.0;
            double cv_val = (dt.vertices[tri.v[0]].y + dt.vertices[tri.v[1]].y + dt.vertices[tri.v[2]].y) / 3.0;
            if (!point_in_polygon(cu, cv_val, trim_poly)) {
                tri.alive = false;
                continue;
            }
            for (const auto& hp : hole_polys) {
                if (point_in_polygon(cu, cv_val, hp)) { tri.alive = false; break; }
            }
        }
    } else {
        int c0 = insert_and_cache(u_min, v_min);
        int c1 = insert_and_cache(u_max, v_min);
        int c2 = insert_and_cache(u_max, v_max);
        int c3 = insert_and_cache(u_min, v_max);

        std::vector<double> u_spans = m_surface.get_span_vector(0);
        std::vector<double> v_spans = m_surface.get_span_vector(1);

        std::vector<int> bottom_verts, top_verts;
        bottom_verts.push_back(c0);
        top_verts.push_back(c3);
        for (size_t i = 1; i < u_spans.size() - 1; ++i) {
            bottom_verts.push_back(insert_and_cache(u_spans[i], v_min));
            top_verts.push_back(insert_and_cache(u_spans[i], v_max));
        }
        bottom_verts.push_back(c1);
        top_verts.push_back(c2);

        std::vector<int> left_verts, right_verts;
        left_verts.push_back(c0);
        right_verts.push_back(c1);
        for (size_t i = 1; i < v_spans.size() - 1; ++i) {
            left_verts.push_back(insert_and_cache(u_min, v_spans[i]));
            right_verts.push_back(insert_and_cache(u_max, v_spans[i]));
        }
        left_verts.push_back(c3);
        right_verts.push_back(c2);

        for (size_t i = 1; i < u_spans.size() - 1; ++i)
            for (size_t j = 1; j < v_spans.size() - 1; ++j)
                insert_and_cache(u_spans[i], v_spans[j]);

        auto constrain_chain = [&](const std::vector<int>& verts) {
            for (size_t i = 0; i + 1 < verts.size(); ++i) {
                if (verts[i] >= 0 && verts[i + 1] >= 0)
                    dt.insert_constraint(verts[i], verts[i + 1]);
            }
        };
        constrain_chain(bottom_verts);
        constrain_chain(right_verts);
        std::vector<int> top_rev(top_verts.rbegin(), top_verts.rend());
        constrain_chain(top_rev);
        std::vector<int> left_rev(left_verts.rbegin(), left_verts.rend());
        constrain_chain(left_rev);

        dt.cleanup();

        for (auto& tri : dt.triangles) {
            if (!tri.alive) continue;
            double cu = (dt.vertices[tri.v[0]].x + dt.vertices[tri.v[1]].x + dt.vertices[tri.v[2]].x) / 3.0;
            double cv_val = (dt.vertices[tri.v[0]].y + dt.vertices[tri.v[1]].y + dt.vertices[tri.v[2]].y) / 3.0;
            double eps = 1e-10;
            if (cu < u_min - eps || cu > u_max + eps || cv_val < v_min - eps || cv_val > v_max + eps)
                tri.alive = false;
        }
    }

    FlatMap64<double> edge_chord_cache;
    edge_chord_cache.reserve(4096);

    double max_edge_sq = max_edge * max_edge;
    double min_edge_sq = min_edge * min_edge;
    double chord_tol_sq = chord_tol * chord_tol;
    double cos_angle = std::cos(angle_rad);

    std::priority_queue<TriScore> pq;

    if (!m_surface.is_planar()) {
        for (int ti = 0; ti < (int)dt.triangles.size(); ++ti) {
            if (!dt.triangles[ti].alive) continue;
            double s = score_triangle(ti, dt, pos_cache, nrm_cache, edge_chord_cache,
                                      m_surface, max_edge_sq, chord_tol_sq, cos_angle, min_edge_sq,
                                      max_edge, chord_tol, angle_rad);
            if (s > 0) pq.push({s, ti});
        }
    }

    for (int iter = 0; iter < m_max_iterations && !m_surface.is_planar(); ++iter) {
        int worst = -1;
        while (!pq.empty()) {
            auto top = pq.top(); pq.pop();
            if (dt.triangles[top.tri_idx].alive) {
                worst = top.tri_idx;
                break;
            }
        }
        if (worst < 0) break;

        dt.last_found_ = worst;
        const Triangle& wtri = dt.triangles[worst];
        double cu, cv_val;
        Delaunay2D::circumcenter(
            dt.vertices[wtri.v[0]].x, dt.vertices[wtri.v[0]].y,
            dt.vertices[wtri.v[1]].x, dt.vertices[wtri.v[1]].y,
            dt.vertices[wtri.v[2]].x, dt.vertices[wtri.v[2]].y,
            cu, cv_val);

        cu = std::max(u_min, std::min(u_max, cu));
        cv_val = std::max(v_min, std::min(v_max, cv_val));

        if (trimmed) {
            if (!point_in_polygon(cu, cv_val, trim_poly)) continue;
            bool in_hole = false;
            for (const auto& hp : hole_polys) {
                if (point_in_polygon(cu, cv_val, hp)) { in_hole = true; break; }
            }
            if (in_hole) continue;
        }

        int old_count = (int)dt.vertices.size();
        int tri_before = (int)dt.triangles.size();
        int new_v = dt.insert(cu, cv_val);
        if (new_v < 0) break;
        if ((int)dt.vertices.size() == old_count) continue;

        cache_vertex(new_v);

        for (int ti = tri_before; ti < (int)dt.triangles.size(); ++ti) {
            Triangle& tri = dt.triangles[ti];
            if (!tri.alive) continue;
            double tc_u = (dt.vertices[tri.v[0]].x + dt.vertices[tri.v[1]].x + dt.vertices[tri.v[2]].x) / 3.0;
            double tc_v = (dt.vertices[tri.v[0]].y + dt.vertices[tri.v[1]].y + dt.vertices[tri.v[2]].y) / 3.0;
            if (trimmed) {
                if (!point_in_polygon(tc_u, tc_v, trim_poly)) { tri.alive = false; continue; }
                for (const auto& hp : hole_polys) {
                    if (point_in_polygon(tc_u, tc_v, hp)) { tri.alive = false; break; }
                }
            } else {
                double eps = 1e-10;
                if (tc_u < u_min - eps || tc_u > u_max + eps || tc_v < v_min - eps || tc_v > v_max + eps)
                    tri.alive = false;
            }
        }

        for (int ti = tri_before; ti < (int)dt.triangles.size(); ++ti) {
            if (!dt.triangles[ti].alive) continue;
            double s = score_triangle(ti, dt, pos_cache, nrm_cache, edge_chord_cache,
                                      m_surface, max_edge_sq, chord_tol_sq, cos_angle, min_edge_sq,
                                      max_edge, chord_tol, angle_rad);
            if (s > 0) pq.push({s, ti});
        }
    }

    // Build output Mesh
    Mesh result;
    FlatMap64<size_t> vertex_map;

    auto tris = dt.get_triangles();
    for (const auto& tri : tris) {
        for (int k = 0; k < 3; ++k) {
            int vi = tri[k];
            if (!vertex_map.find((uint64_t)vi)) {
                const Pos3& pt = pos_cache[vi];
                const Pos3& n = nrm_cache[vi];
                size_t vk = result.add_vertex(Point(pt[0], pt[1], pt[2]));
                result.vertex[vk].set_normal(n[0], n[1], n[2]);
                vertex_map[(uint64_t)vi] = vk;
            }
        }
        std::vector<size_t> face_verts = {
            vertex_map[(uint64_t)tri[0]],
            vertex_map[(uint64_t)tri[1]],
            vertex_map[(uint64_t)tri[2]]
        };
        result.add_face(face_verts);
    }

    return result;
}

} // namespace session_cpp
