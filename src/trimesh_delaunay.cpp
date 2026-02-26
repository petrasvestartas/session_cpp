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
                if (tri.constrained[k]) continue;
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
                bool is_c = tri.constrained[k];
                polygon_.push_back({tri.v[(k + 1) % 3], tri.v[(k + 2) % 3], is_c});
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
        nt.constrained[0] = edge.constrained;
        nt.adj[0] = -1; nt.adj[1] = -1; nt.adj[2] = -1;
        nt.alive = true;
        triangles.push_back(nt);
        register_triangle_edges(new_ti);
    }

    last_found_ = (int)triangles.size() - 1;

    return vi;
}

static int find_slot(const Triangle& t, int v) {
    for (int k = 0; k < 3; ++k) if (t.v[k] == v) return k;
    return -1;
}

static int opposed_vertex(const Triangle& tOpo, int from_ti) {
    for (int k = 0; k < 3; ++k) if (tOpo.adj[k] == from_ti) return tOpo.v[k];
    return -1;
}

static bool tri_contains(const Triangle& t, int v) {
    return t.v[0] == v || t.v[1] == v || t.v[2] == v;
}

static void mark_edge_constrained(std::vector<Triangle>& tris, int v0, int v1) {
    for (auto& tri : tris) {
        if (!tri.alive) continue;
        for (int k = 0; k < 3; ++k) {
            int e0 = tri.v[(k + 1) % 3], e1 = tri.v[(k + 2) % 3];
            if ((e0 == v0 && e1 == v1) || (e0 == v1 && e1 == v0))
                tri.constrained[k] = true;
        }
    }
}

bool Delaunay2D::flip_edge(int, int) { return false; } // unused

void Delaunay2D::insert_constraint(int v0, int v1) {
    if (v0 == v1) return;

    // --- Step 1: check if edge already exists (linear scan) ---
    for (int ti = 0; ti < (int)triangles.size(); ++ti) {
        if (!triangles[ti].alive) continue;
        for (int k = 0; k < 3; ++k) {
            int e0 = triangles[ti].v[(k + 1) % 3], e1 = triangles[ti].v[(k + 2) % 3];
            if ((e0 == v0 && e1 == v1) || (e0 == v1 && e1 == v0)) {
                triangles[ti].constrained[k] = true;
                int nb = triangles[ti].adj[k];
                if (nb >= 0 && triangles[nb].alive)
                    for (int kk = 0; kk < 3; ++kk)
                        if (triangles[nb].adj[kk] == ti) { triangles[nb].constrained[kk] = true; break; }
                return;
            }
        }
    }

    // --- Step 2: fan walk around v0 to find the first straddled triangle ---
    // Find any triangle containing v0
    int start_ti = -1;
    for (int i = (int)triangles.size() - 1; i >= 0; --i)
        if (triangles[i].alive)
            for (int k = 0; k < 3; ++k)
                if (triangles[i].v[k] == v0) { start_ti = i; break; }
    if (start_ti < 0) return;

    const Vertex2D& a = vertices[v0];
    const Vertex2D& b = vertices[v1];

    // Walk CW around v0's fan: at slot k of v0, the CW-next triangle is adj[(k+1)%3]
    // Look for triangle where CCW-neighbor (v[(k+1)%3]) is Right of v0→v1
    //   and  CW-neighbor  (v[(k+2)%3]) is Left  of v0→v1
    int iVL = -1, iVR = -1, iT = -1;
    {
        int ti = start_ti;
        int guard = (int)triangles.size() + 4;
        do {
            if (!triangles[ti].alive) break;
            const Triangle& t = triangles[ti];
            int k = find_slot(t, v0);
            if (k < 0) break;
            int iP2 = t.v[(k + 1) % 3]; // CCW from v0
            int iP1 = t.v[(k + 2) % 3]; // CW from v0
            double oP2 = orient2d(a.x, a.y, b.x, b.y, vertices[iP2].x, vertices[iP2].y);
            double oP1 = orient2d(a.x, a.y, b.x, b.y, vertices[iP1].x, vertices[iP1].y);
            // Straddled: P2 strictly Right, P1 Left-or-on
            if (oP2 < 0 && oP1 >= 0) { iVL = iP1; iVR = iP2; iT = ti; break; }
            int next = t.adj[(k + 1) % 3]; // CW fan step
            if (next < 0 || !triangles[next].alive) break;
            ti = next;
        } while (ti != start_ti && --guard > 0);
    }
    if (iT < 0) return;

    // --- Step 3: walk across all intersected triangles (CDT cavity walk) ---
    // iV tracks which vertex of cur_iT is "behind" us (the vertex whose opposite
    // triangle is the next one to cross).  Start with iV=v0 so we cross edge (iVL,iVR).
    std::vector<int> polyL = {v0, iVL}; // left boundary chain
    std::vector<int> polyR = {v0, iVR}; // right boundary chain
    std::vector<int> intersected = {iT};
    int iV = v0, cur_iT = iT;

    int guard = (int)triangles.size() * 2 + 8;
    while (!tri_contains(triangles[cur_iT], v1) && --guard > 0) {
        const Triangle& t = triangles[cur_iT];
        int k_iV = find_slot(t, iV);
        if (k_iV < 0) break;
        int iTopo = t.adj[k_iV]; // cross edge opposite iV
        if (iTopo < 0 || !triangles[iTopo].alive) break;
        int iVopo = opposed_vertex(triangles[iTopo], cur_iT);
        if (iVopo < 0) break;

        double o = orient2d(a.x, a.y, b.x, b.y, vertices[iVopo].x, vertices[iVopo].y);
        if (o < 0) { // Right of v0→v1
            if (iVopo != v1) polyR.push_back(iVopo);
            iV = iVR;
            iVR = iVopo;
        } else {     // Left or on line
            if (iVopo != v1) polyL.push_back(iVopo);
            iV = iVL;
            iVL = iVopo;
        }
        intersected.push_back(iTopo);
        cur_iT = iTopo;
    }
    polyL.push_back(v1);
    polyR.push_back(v1);

    // --- Step 4: kill all intersected triangles ---
    for (int ti : intersected) {
        unregister_triangle_edges(ti);
        triangles[ti].alive = false;
    }

    // --- Step 5: fan-triangulate each cavity half from the shared endpoints ---
    // Left cavity:  polyL = [v0, L1..Lk, v1]  → fan from v1 (gives CCW triangles)
    // Right cavity: polyR = [v0, R1..Rm, v1]  → fan from v0 (gives CCW triangles)
    // For both, let orient decide and swap v[1]/v[2] if CW.
    int first_new = (int)triangles.size();

    auto emit = [&](int pa, int pb, int pc) {
        double o = orient2d(vertices[pa].x, vertices[pa].y,
                            vertices[pb].x, vertices[pb].y,
                            vertices[pc].x, vertices[pc].y);
        if (std::abs(o) < 1e-20) return;
        Triangle nt;
        nt.v[0] = pa;
        if (o > 0) { nt.v[1] = pb; nt.v[2] = pc; }
        else       { nt.v[1] = pc; nt.v[2] = pb; }
        nt.adj[0] = nt.adj[1] = nt.adj[2] = -1;
        nt.constrained[0] = nt.constrained[1] = nt.constrained[2] = false;
        nt.alive = true;
        int new_ti = (int)triangles.size();
        triangles.push_back(nt);
        register_triangle_edges(new_ti);
    };

    // Left fan from v1 (fan apex = v1 = polyL.back())
    {
        int apex = v1;
        for (int i = 0; i + 2 < (int)polyL.size(); ++i)
            emit(apex, polyL[i + 1], polyL[i]);
    }
    // Right fan from v0 (fan apex = v0 = polyR[0], skip i=0 which is apex itself)
    {
        int apex = v0;
        for (int i = 1; i + 1 < (int)polyR.size(); ++i)
            emit(apex, polyR[i], polyR[i + 1]);
    }

    // --- Step 6: propagate constrained flags from outer neighbors ---
    for (int new_ti = first_new; new_ti < (int)triangles.size(); ++new_ti) {
        if (!triangles[new_ti].alive) continue;
        Triangle& nt = triangles[new_ti];
        for (int k = 0; k < 3; ++k) {
            int nb = nt.adj[k];
            if (nb < 0 || nb >= first_new || !triangles[nb].alive) continue;
            Triangle& nb_t = triangles[nb];
            for (int kk = 0; kk < 3; ++kk)
                if (nb_t.adj[kk] == new_ti && nb_t.constrained[kk])
                    { nt.constrained[k] = true; break; }
        }
    }

    // --- Step 7: mark constraint edge ---
    mark_edge_constrained(triangles, v0, v1);
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

    bool closed_u = m_surface.is_closed(0);
    bool closed_v = m_surface.is_closed(1);
    bool sing_south = m_surface.is_singular(0);
    bool sing_north = m_surface.is_singular(2);

    double bbox_diag = compute_bbox_diagonal();
    double min_edge = m_min_edge_length > 0 ? m_min_edge_length : bbox_diag / 1000.0;
    double chord_tol = m_max_chord_height > 0 ? m_max_chord_height : bbox_diag / 500.0;
    double angle_rad = m_max_angle * Tolerance::PI / 180.0;

    // Curvature-driven span subdivisions (normal-angle + chord-height)
    std::vector<double> usp = m_surface.get_span_vector(0);
    std::vector<double> vsp = m_surface.get_span_vector(1);
    int deg_u = m_surface.degree(0), deg_v = m_surface.degree(1);
    double max_angle_deg = m_max_angle;

    auto span_subs = [&](int dir, const std::vector<double>& sp,
                         const std::vector<double>& osp) -> std::vector<int> {
        int n = (int)sp.size() - 1;
        std::vector<int> subs(n, 1);
        int n_other = (int)osp.size() - 1;
        std::vector<double> s_positions(n_other);
        for (int k = 0; k < n_other; ++k)
            s_positions[k] = (osp[k] + osp[k + 1]) * 0.5;
        int degree_dir = (dir == 0) ? deg_u : deg_v;
        for (int i = 0; i < n; ++i) {
            double t0 = sp[i], t1 = sp[i + 1];
            if (degree_dir > 1) {
                double ma = 0.0;
                for (int si = 0; si < n_other; ++si) {
                    double s = s_positions[si];
                    double prev_nx = 0, prev_ny = 0, prev_nz = 0;
                    double total_angle = 0.0;
                    for (int k = 0; k <= 4; ++k) {
                        double t = t0 + k * (t1 - t0) / 4.0;
                        Vector nrm = (dir == 0) ? m_surface.normal_at(t, s) : m_surface.normal_at(s, t);
                        double nx = nrm[0], ny = nrm[1], nz = nrm[2];
                        if (k > 0) {
                            double dot = prev_nx*nx + prev_ny*ny + prev_nz*nz;
                            dot = std::max(-1.0, std::min(1.0, dot));
                            total_angle += std::acos(dot) * 180.0 / Tolerance::PI;
                        }
                        prev_nx = nx; prev_ny = ny; prev_nz = nz;
                    }
                    if (total_angle > ma) ma = total_angle;
                }
                subs[i] = std::max(1, std::min((int)std::ceil(ma / max_angle_deg), 24));
            }
            {
                double ct = (m_max_chord_height > 0) ? m_max_chord_height : bbox_diag * 0.005;
                double max_dev = 0.0;
                int nc = std::min(n_other, 3);
                for (int ci = 0; ci <= nc; ++ci) {
                    double s = osp.front() + ci * (osp.back() - osp.front()) / std::max(nc, 1);
                    double px0, py0, pz0, px1, py1, pz1;
                    if (dir == 0) {
                        m_surface.point_at(t0, s, px0, py0, pz0);
                        m_surface.point_at(t1, s, px1, py1, pz1);
                    } else {
                        m_surface.point_at(s, t0, px0, py0, pz0);
                        m_surface.point_at(s, t1, px1, py1, pz1);
                    }
                    for (int k = 1; k <= 3; ++k) {
                        double frac = k / 4.0;
                        double tm = t0 + frac * (t1 - t0);
                        double pmx, pmy, pmz;
                        if (dir == 0) m_surface.point_at(tm, s, pmx, pmy, pmz);
                        else          m_surface.point_at(s, tm, pmx, pmy, pmz);
                        double lx = px0 + frac * (px1 - px0);
                        double ly = py0 + frac * (py1 - py0);
                        double lz = pz0 + frac * (pz1 - pz0);
                        double dx = pmx - lx, dy = pmy - ly, dz = pmz - lz;
                        double dev = std::sqrt(dx*dx + dy*dy + dz*dz);
                        if (dev > max_dev) max_dev = dev;
                    }
                }
                if (max_dev > ct) {
                    int chord_subs = std::max(2, (int)std::ceil(std::sqrt(max_dev / ct)));
                    subs[i] = std::max(subs[i], std::min(chord_subs, 24));
                }
            }
            if (m_max_edge_length > 0) {
                double s_mid = (osp.front() + osp.back()) * 0.5;
                double px0, py0, pz0, px1, py1, pz1;
                if (dir == 0) {
                    m_surface.point_at(t0, s_mid, px0, py0, pz0);
                    m_surface.point_at(t1, s_mid, px1, py1, pz1);
                } else {
                    m_surface.point_at(s_mid, t0, px0, py0, pz0);
                    m_surface.point_at(s_mid, t1, px1, py1, pz1);
                }
                double span_len = std::sqrt((px1-px0)*(px1-px0)+(py1-py0)*(py1-py0)+(pz1-pz0)*(pz1-pz0));
                int edge_subs = std::max(1, (int)std::ceil(span_len / m_max_edge_length));
                subs[i] = std::max(subs[i], std::min(edge_subs, 64));
            }
            if (degree_dir > 1) subs[i] = std::max(subs[i], 2);
        }
        return subs;
    };

    std::vector<int> u_subs = span_subs(0, usp, vsp);
    std::vector<int> v_subs = span_subs(1, vsp, usp);

    // Aspect ratio balancing
    {
        int total_u = 0, total_v = 0;
        for (int s : u_subs) total_u += s;
        for (int s : v_subs) total_v += s;
        total_u += 1; total_v += 1;
        double v_mid = (vsp.front() + vsp.back()) * 0.5;
        double u_mid = (usp.front() + usp.back()) * 0.5;
        double u_len = 0.0, v_len = 0.0;
        {
            double px0, py0, pz0;
            m_surface.point_at(usp.front(), v_mid, px0, py0, pz0);
            int n_sample = std::max(total_u, 10);
            for (int i = 1; i <= n_sample; ++i) {
                double u = usp.front() + i * (usp.back() - usp.front()) / n_sample;
                double px1, py1, pz1;
                m_surface.point_at(u, v_mid, px1, py1, pz1);
                u_len += std::sqrt((px1-px0)*(px1-px0)+(py1-py0)*(py1-py0)+(pz1-pz0)*(pz1-pz0));
                px0 = px1; py0 = py1; pz0 = pz1;
            }
        }
        {
            double px0, py0, pz0;
            m_surface.point_at(u_mid, vsp.front(), px0, py0, pz0);
            int n_sample = std::max(total_v, 10);
            for (int i = 1; i <= n_sample; ++i) {
                double v = vsp.front() + i * (vsp.back() - vsp.front()) / n_sample;
                double px1, py1, pz1;
                m_surface.point_at(u_mid, v, px1, py1, pz1);
                v_len += std::sqrt((px1-px0)*(px1-px0)+(py1-py0)*(py1-py0)+(pz1-pz0)*(pz1-pz0));
                px0 = px1; py0 = py1; pz0 = pz1;
            }
        }
        if (u_len > 1e-14 && v_len > 1e-14 && total_u > 0 && total_v > 0) {
            double spacing_u = u_len / total_u;
            double spacing_v = v_len / total_v;
            double ratio = spacing_u / spacing_v;
            if (ratio > 2.0 && deg_u > 1) {
                double scale = std::sqrt(ratio);
                for (int& s : u_subs) s = std::min((int)std::ceil(s * scale), 24);
            } else if (ratio < 0.5 && deg_v > 1) {
                double scale = std::sqrt(1.0 / ratio);
                for (int& s : v_subs) s = std::min((int)std::ceil(s * scale), 24);
            }
        }
    }

    // Bilinear twist check
    int ns_u = (int)usp.size() - 1, ns_v = (int)vsp.size() - 1;
    if (deg_u == 1 && deg_v == 1) {
        double twist_tol = (bbox_diag > 0) ? bbox_diag * 0.005 : 1e-6;
        double max_twist = 0.0;
        for (int i = 0; i < ns_u; ++i)
            for (int j = 0; j < ns_v; ++j) {
                double u0 = usp[i], u1 = usp[i+1];
                double v0 = vsp[j], v1 = vsp[j+1];
                double pmx, pmy, pmz;
                m_surface.point_at((u0+u1)*0.5, (v0+v1)*0.5, pmx, pmy, pmz);
                double p00x, p00y, p00z, p11x, p11y, p11z;
                m_surface.point_at(u0, v0, p00x, p00y, p00z);
                m_surface.point_at(u1, v1, p11x, p11y, p11z);
                double mx = (p00x+p11x)*0.5, my = (p00y+p11y)*0.5, mz = (p00z+p11z)*0.5;
                double dx = pmx-mx, dy = pmy-my, dz = pmz-mz;
                double twist = std::sqrt(dx*dx+dy*dy+dz*dz);
                if (twist > max_twist) max_twist = twist;
            }
        if (max_twist > twist_tol) {
            int twist_subs = std::max(4, std::min((int)std::ceil(2.0 * std::sqrt(max_twist / twist_tol)), 24));
            for (int& s : u_subs) s = std::max(s, twist_subs);
            for (int& s : v_subs) s = std::max(s, twist_subs);
        }
    }

    // Build parameter arrays (full UV rectangle)
    std::vector<double> us, vs;
    for (int i = 0; i < ns_u; ++i)
        for (int s = 0; s < u_subs[i]; ++s)
            us.push_back(usp[i] + s * (usp[i+1] - usp[i]) / u_subs[i]);
    us.push_back(usp.back());
    for (int i = 0; i < ns_v; ++i)
        for (int s = 0; s < v_subs[i]; ++s)
            vs.push_back(vsp[i] + s * (vsp[i+1] - vsp[i]) / v_subs[i]);
    vs.push_back(vsp.back());

    int nu = (int)us.size(), nv = (int)vs.size();

    // Pole rows: exclude from CDT, handle explicitly
    int j_start = sing_south ? 1 : 0;
    int j_end = sing_north ? nv - 1 : nv;

    // CDT in non-pole UV domain
    double cdt_v_lo = vs[j_start];
    double cdt_v_hi = vs[j_end - 1];
    Delaunay2D dt(u_min, cdt_v_lo, u_max, cdt_v_hi);

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
            Point p = m_surface.point_at(u, v);
            Vector n = m_surface.normal_at(u, v);
            pos_cache[vi] = {p[0], p[1], p[2]};
            nrm_cache[vi] = {n[0], n[1], n[2]};
        }
    };

    for (int k = 0; k < 3; ++k) cache_vertex(dt.super_v[k]);

    auto insert_and_cache = [&](double u, double v) -> int {
        int vi = dt.insert(u, v);
        if (vi >= 0) cache_vertex(vi);
        return vi;
    };

    // Insert grid vertices (excluding pole rows)
    std::vector<std::vector<int>> grid_vi(nu, std::vector<int>(nv, -1));
    for (int i = 0; i < nu; ++i)
        for (int j = j_start; j < j_end; ++j)
            grid_vi[i][j] = insert_and_cache(us[i], vs[j]);

    // Constrain all 4 boundary edges
    auto constrain_chain = [&](const std::vector<int>& verts) {
        for (size_t i = 0; i + 1 < verts.size(); ++i)
            if (verts[i] >= 0 && verts[i + 1] >= 0)
                dt.insert_constraint(verts[i], verts[i + 1]);
    };
    {
        std::vector<int> bottom, top, left, right;
        for (int i = 0; i < nu; ++i) bottom.push_back(grid_vi[i][j_start]);
        for (int i = nu - 1; i >= 0; --i) top.push_back(grid_vi[i][j_end - 1]);
        for (int j = j_start; j < j_end; ++j) left.push_back(grid_vi[0][j]);
        for (int j = j_end - 1; j >= j_start; --j) right.push_back(grid_vi[nu - 1][j]);
        constrain_chain(bottom);
        constrain_chain(top);
        constrain_chain(left);
        constrain_chain(right);
    }

    dt.cleanup();

    // Cull triangles outside CDT domain
    for (auto& tri : dt.triangles) {
        if (!tri.alive) continue;
        double cu = (dt.vertices[tri.v[0]].x + dt.vertices[tri.v[1]].x + dt.vertices[tri.v[2]].x) / 3.0;
        double cv_val = (dt.vertices[tri.v[0]].y + dt.vertices[tri.v[1]].y + dt.vertices[tri.v[2]].y) / 3.0;
        double eps = 1e-10;
        if (cu < u_min - eps || cu > u_max + eps || cv_val < cdt_v_lo - eps || cv_val > cdt_v_hi + eps)
            tri.alive = false;
    }

    // Refinement — curvature-driven (no edge-length unless user-specified)
    FlatMap64<double> edge_chord_cache;
    edge_chord_cache.reserve(4096);

    double refine_max_edge, refine_max_edge_sq;
    if (m_max_edge_length > 0) {
        refine_max_edge = m_max_edge_length;
        refine_max_edge_sq = refine_max_edge * refine_max_edge;
    } else {
        refine_max_edge = 1e30;
        refine_max_edge_sq = 1e60;
    }
    double min_edge_sq = min_edge * min_edge;
    double chord_tol_sq = chord_tol * chord_tol;
    double cos_angle = std::cos(angle_rad);

    // Clamp margins: keep refinement strictly inside boundaries
    double u_margin = (u_max - u_min) * 1e-4;
    double v_margin = (cdt_v_hi - cdt_v_lo) * 1e-4;
    double clamp_u_lo = u_min + u_margin;
    double clamp_u_hi = u_max - u_margin;
    double clamp_v_lo = cdt_v_lo + v_margin;
    double clamp_v_hi = cdt_v_hi - v_margin;

    std::priority_queue<TriScore> pq;

    if (!m_surface.is_planar()) {
        for (int ti = 0; ti < (int)dt.triangles.size(); ++ti) {
            if (!dt.triangles[ti].alive) continue;
            double s = score_triangle(ti, dt, pos_cache, nrm_cache, edge_chord_cache,
                                      m_surface, refine_max_edge_sq, chord_tol_sq, cos_angle, min_edge_sq,
                                      refine_max_edge, chord_tol, angle_rad);
            if (s > 0) pq.push({s, ti});
        }
    }

    for (int iter = 0; iter < m_max_iterations && !m_surface.is_planar(); ++iter) {
        int worst = -1;
        while (!pq.empty()) {
            auto top_item = pq.top(); pq.pop();
            if (dt.triangles[top_item.tri_idx].alive) {
                worst = top_item.tri_idx;
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

        cu = std::max(clamp_u_lo, std::min(clamp_u_hi, cu));
        cv_val = std::max(clamp_v_lo, std::min(clamp_v_hi, cv_val));

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
            double eps = 1e-10;
            if (tc_u < u_min - eps || tc_u > u_max + eps || tc_v < cdt_v_lo - eps || tc_v > cdt_v_hi + eps)
                tri.alive = false;
        }

        for (int ti = tri_before; ti < (int)dt.triangles.size(); ++ti) {
            if (!dt.triangles[ti].alive) continue;
            double s = score_triangle(ti, dt, pos_cache, nrm_cache, edge_chord_cache,
                                      m_surface, refine_max_edge_sq, chord_tol_sq, cos_angle, min_edge_sq,
                                      refine_max_edge, chord_tol, angle_rad);
            if (s > 0) pq.push({s, ti});
        }
    }

    // Build output mesh — explicit seam mapping (no 3D-distance merge)
    Mesh result;

    // Seam map: right boundary → left boundary for closed surfaces
    FlatMap64<int> seam_map;
    if (closed_u) {
        for (int j = j_start; j < j_end; ++j) {
            int right_vi = grid_vi[nu - 1][j];
            int left_vi = grid_vi[0][j];
            if (right_vi >= 0 && left_vi >= 0 && right_vi != left_vi)
                seam_map[(uint64_t)right_vi] = left_vi;
        }
    }
    if (closed_v && !sing_south && !sing_north) {
        for (int i = 0; i < nu; ++i) {
            int top_vi = grid_vi[i][j_end - 1];
            int bot_vi = grid_vi[i][j_start];
            if (top_vi >= 0 && bot_vi >= 0 && top_vi != bot_vi) {
                // Follow existing chain for canonical target
                int canonical = bot_vi;
                int* m;
                while ((m = seam_map.find((uint64_t)canonical)) != nullptr) canonical = *m;
                seam_map[(uint64_t)top_vi] = canonical;
            }
        }
    }

    auto resolve_vi = [&](int vi) -> int {
        for (int i = 0; i < 4; ++i) {
            int* m = seam_map.find((uint64_t)vi);
            if (!m) break;
            vi = *m;
        }
        return vi;
    };

    FlatMap64<size_t> vertex_map;

    auto get_or_create_vertex = [&](int vi) -> size_t {
        vi = resolve_vi(vi);
        size_t* existing = vertex_map.find((uint64_t)vi);
        if (existing) return *existing;

        const Pos3& pt = pos_cache[vi];
        const Pos3& n = nrm_cache[vi];
        size_t vk = result.add_vertex(Point(pt[0], pt[1], pt[2]));
        result.vertex[vk].set_normal(n[0], n[1], n[2]);
        vertex_map[(uint64_t)vi] = vk;
        return vk;
    };

    // Pole vertices
    size_t south_pole_vk = 0, north_pole_vk = 0;
    if (sing_south) {
        Point p = m_surface.point_at(us[0], vs[0]);
        Vector n = m_surface.normal_at(us[0], vs[0]);
        south_pole_vk = result.add_vertex(p);
        result.vertex[south_pole_vk].set_normal(n[0], n[1], n[2]);
    }
    if (sing_north) {
        Point p = m_surface.point_at(us[0], vs[nv - 1]);
        Vector n = m_surface.normal_at(us[0], vs[nv - 1]);
        north_pole_vk = result.add_vertex(p);
        result.vertex[north_pole_vk].set_normal(n[0], n[1], n[2]);
    }

    // CDT interior triangles
    auto tris = dt.get_triangles();
    for (const auto& tri : tris) {
        size_t vk0 = get_or_create_vertex(tri[0]);
        size_t vk1 = get_or_create_vertex(tri[1]);
        size_t vk2 = get_or_create_vertex(tri[2]);
        if (vk0 == vk1 || vk1 == vk2 || vk2 == vk0) continue;
        result.add_face({vk0, vk1, vk2});
    }

    // Pole fan triangles
    if (sing_south) {
        for (int i = 0; i < nu - 1; ++i) {
            size_t v0 = get_or_create_vertex(grid_vi[i][j_start]);
            size_t v1 = get_or_create_vertex(grid_vi[i + 1][j_start]);
            if (v0 != v1)
                result.add_face({south_pole_vk, v1, v0});
        }
    }
    if (sing_north) {
        int j_last = j_end - 1;
        for (int i = 0; i < nu - 1; ++i) {
            size_t v0 = get_or_create_vertex(grid_vi[i][j_last]);
            size_t v1 = get_or_create_vertex(grid_vi[i + 1][j_last]);
            if (v0 != v1)
                result.add_face({v0, v1, north_pole_vk});
        }
    }

    return result;
}

} // namespace session_cpp
