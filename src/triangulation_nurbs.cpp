#include "triangulation_nurbs.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

namespace session_cpp {

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

    Triangle t;
    t.v[0] = 0; t.v[1] = 1; t.v[2] = 2;
    t.adj[0] = -1; t.adj[1] = -1; t.adj[2] = -1;
    t.alive = true;
    triangles.push_back(t);
}

void Delaunay2D::update_adjacency(int tri_idx, int va, int vb, int new_neighbor) {
    Triangle& tri = triangles[tri_idx];
    for (int k = 0; k < 3; ++k) {
        int e0 = tri.v[(k + 1) % 3];
        int e1 = tri.v[(k + 2) % 3];
        if ((e0 == va && e1 == vb) || (e0 == vb && e1 == va)) {
            tri.adj[k] = new_neighbor;
            return;
        }
    }
}

void Delaunay2D::link_new_triangles(const std::vector<int>& new_tris) {
    for (size_t i = 0; i < new_tris.size(); ++i) {
        for (size_t j = i + 1; j < new_tris.size(); ++j) {
            Triangle& ti = triangles[new_tris[i]];
            Triangle& tj = triangles[new_tris[j]];
            for (int ki = 0; ki < 3; ++ki) {
                int ei0 = ti.v[(ki + 1) % 3];
                int ei1 = ti.v[(ki + 2) % 3];
                for (int kj = 0; kj < 3; ++kj) {
                    int ej0 = tj.v[(kj + 1) % 3];
                    int ej1 = tj.v[(kj + 2) % 3];
                    if ((ei0 == ej1 && ei1 == ej0) || (ei0 == ej0 && ei1 == ej1)) {
                        ti.adj[ki] = new_tris[j];
                        tj.adj[kj] = new_tris[i];
                    }
                }
            }
        }
    }
}

int Delaunay2D::find_triangle_containing(double x, double y) const {
    for (int i = static_cast<int>(triangles.size()) - 1; i >= 0; --i) {
        if (!triangles[i].alive) continue;
        const Triangle& tri = triangles[i];
        const Vertex2D& a = vertices[tri.v[0]];
        const Vertex2D& b = vertices[tri.v[1]];
        const Vertex2D& c = vertices[tri.v[2]];
        double d0 = orient2d(a.x, a.y, b.x, b.y, x, y);
        double d1 = orient2d(b.x, b.y, c.x, c.y, x, y);
        double d2 = orient2d(c.x, c.y, a.x, a.y, x, y);
        if (d0 >= -1e-12 && d1 >= -1e-12 && d2 >= -1e-12) return i;
    }
    return -1;
}

int Delaunay2D::insert(double x, double y) {
    // Check for duplicate vertex
    for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
        double dx = vertices[i].x - x;
        double dy = vertices[i].y - y;
        if (dx * dx + dy * dy < 1e-20) return i;
    }

    int vi = static_cast<int>(vertices.size());
    vertices.push_back({x, y});

    // Find bad triangles (circumcircle contains new point)
    std::vector<int> bad;
    for (int ti = 0; ti < static_cast<int>(triangles.size()); ++ti) {
        if (!triangles[ti].alive) continue;
        const Triangle& tri = triangles[ti];
        const Vertex2D& a = vertices[tri.v[0]];
        const Vertex2D& b = vertices[tri.v[1]];
        const Vertex2D& c = vertices[tri.v[2]];

        // Ensure CCW orientation for circumcircle test
        double o = orient2d(a.x, a.y, b.x, b.y, c.x, c.y);
        double ic;
        if (o > 0) {
            ic = in_circumcircle(a.x, a.y, b.x, b.y, c.x, c.y, x, y);
        } else {
            ic = in_circumcircle(a.x, a.y, c.x, c.y, b.x, b.y, x, y);
        }
        if (ic > 0) bad.push_back(ti);
    }

    if (bad.empty()) {
        vertices.pop_back();
        return -1;
    }

    // Find boundary polygon
    std::unordered_set<int> bad_set(bad.begin(), bad.end());
    struct BoundaryEdge { int e0, e1, outside; };
    std::vector<BoundaryEdge> polygon;

    for (int ti : bad) {
        const Triangle& tri = triangles[ti];
        for (int k = 0; k < 3; ++k) {
            int neighbor = tri.adj[k];
            if (neighbor < 0 || bad_set.find(neighbor) == bad_set.end()) {
                int e0 = tri.v[(k + 1) % 3];
                int e1 = tri.v[(k + 2) % 3];
                polygon.push_back({e0, e1, neighbor});
            }
        }
    }

    // Remove bad triangles
    for (int ti : bad) {
        triangles[ti].alive = false;
    }

    // Create new triangles from polygon edges to new vertex
    std::vector<int> new_tris;
    for (const auto& edge : polygon) {
        int new_ti = static_cast<int>(triangles.size());
        Triangle nt;
        nt.v[0] = vi;
        nt.v[1] = edge.e0;
        nt.v[2] = edge.e1;
        nt.adj[0] = -1;  // between e0 and e1 (opposite vi) -> not yet linked to sibling
        nt.adj[1] = -1;  // between vi and e1 (opposite e0)
        nt.adj[2] = edge.outside;  // between vi and e0 (opposite e1) -> outside neighbor
        nt.alive = true;

        // The edge opposite vertex v[0]=vi is the edge (v[1]=e0, v[2]=e1)
        // which faces the outside neighbor
        // adj[0] = outside neighbor (edge opposite v[0])
        nt.adj[0] = edge.outside;
        nt.adj[1] = -1;
        nt.adj[2] = -1;

        triangles.push_back(nt);
        new_tris.push_back(new_ti);

        // Update outside neighbor
        if (edge.outside >= 0) {
            update_adjacency(edge.outside, edge.e0, edge.e1, new_ti);
        }
    }

    // Link new triangles to each other
    link_new_triangles(new_tris);

    return vi;
}

void Delaunay2D::insert_constraint(int v0, int v1) {
    if (v0 == v1) return;

    // Find if edge already exists in triangulation
    auto edge_exists = [&](int a, int b) -> bool {
        for (const auto& tri : triangles) {
            if (!tri.alive) continue;
            for (int k = 0; k < 3; ++k) {
                int e0 = tri.v[(k + 1) % 3];
                int e1 = tri.v[(k + 2) % 3];
                if ((e0 == a && e1 == b) || (e0 == b && e1 == a)) return true;
            }
        }
        return false;
    };

    // Mark edge as constrained if it exists
    auto mark_constrained = [&](int a, int b) {
        for (auto& tri : triangles) {
            if (!tri.alive) continue;
            for (int k = 0; k < 3; ++k) {
                int e0 = tri.v[(k + 1) % 3];
                int e1 = tri.v[(k + 2) % 3];
                if ((e0 == a && e1 == b) || (e0 == b && e1 == a)) {
                    tri.constrained[k] = true;
                }
            }
        }
    };

    if (edge_exists(v0, v1)) {
        mark_constrained(v0, v1);
        return;
    }

    // Edge doesn't exist - we need to force it in using edge flips
    // Find triangles intersected by the segment v0-v1
    const Vertex2D& p0 = vertices[v0];
    const Vertex2D& p1 = vertices[v1];

    // Collect intersected edges and flip them
    // Simple iterative approach: keep flipping until edge exists
    for (int iter = 0; iter < 1000; ++iter) {
        if (edge_exists(v0, v1)) {
            mark_constrained(v0, v1);
            return;
        }

        // Find a triangle edge that intersects segment (v0, v1) and flip it
        bool flipped = false;
        for (int ti = 0; ti < static_cast<int>(triangles.size()) && !flipped; ++ti) {
            if (!triangles[ti].alive) continue;
            Triangle& tri = triangles[ti];

            for (int k = 0; k < 3; ++k) {
                if (tri.constrained[k]) continue;
                int neighbor = tri.adj[k];
                if (neighbor < 0 || !triangles[neighbor].alive) continue;

                int ea = tri.v[(k + 1) % 3];
                int eb = tri.v[(k + 2) % 3];

                // Skip if this edge shares a vertex with v0-v1
                if (ea == v0 || ea == v1 || eb == v0 || eb == v1) continue;

                // Check if edge (ea, eb) intersects segment (v0, v1)
                const Vertex2D& a = vertices[ea];
                const Vertex2D& b = vertices[eb];

                double d1 = orient2d(p0.x, p0.y, p1.x, p1.y, a.x, a.y);
                double d2 = orient2d(p0.x, p0.y, p1.x, p1.y, b.x, b.y);
                double d3 = orient2d(a.x, a.y, b.x, b.y, p0.x, p0.y);
                double d4 = orient2d(a.x, a.y, b.x, b.y, p1.x, p1.y);

                if (d1 * d2 < 0 && d3 * d4 < 0) {
                    // Edges intersect - perform edge flip
                    // tri has vertices opposite k (which is ea, eb edge)
                    int vc = tri.v[k]; // vertex opposite the intersected edge in tri
                    Triangle& ntri = triangles[neighbor];

                    // Find the vertex in neighbor opposite the shared edge
                    int vd = -1;
                    int nk = -1;
                    for (int nki = 0; nki < 3; ++nki) {
                        if (ntri.v[(nki + 1) % 3] == eb && ntri.v[(nki + 2) % 3] == ea) {
                            vd = ntri.v[nki];
                            nk = nki;
                            break;
                        }
                        if (ntri.v[(nki + 1) % 3] == ea && ntri.v[(nki + 2) % 3] == eb) {
                            vd = ntri.v[nki];
                            nk = nki;
                            break;
                        }
                    }
                    if (vd < 0) continue;

                    // Check if flip produces valid (non-degenerate) triangles
                    double o1 = orient2d(vertices[vc].x, vertices[vc].y,
                                         vertices[vd].x, vertices[vd].y,
                                         vertices[ea].x, vertices[ea].y);
                    double o2 = orient2d(vertices[vc].x, vertices[vc].y,
                                         vertices[vd].x, vertices[vd].y,
                                         vertices[eb].x, vertices[eb].y);
                    if (o1 * o2 >= 0) continue; // flip would create degenerate triangle

                    // Save old adjacencies
                    int adj_tri_ea = -1, adj_tri_eb = -1;
                    int adj_ntri_ea = -1, adj_ntri_eb = -1;

                    // Find adjacencies for the non-shared edges
                    for (int kk = 0; kk < 3; ++kk) {
                        if (kk != k) {
                            if (tri.v[(kk + 1) % 3] == ea || tri.v[(kk + 2) % 3] == ea) {
                                if (tri.v[(kk + 1) % 3] != eb && tri.v[(kk + 2) % 3] != eb) {
                                    adj_tri_ea = tri.adj[kk];
                                }
                            }
                            if (tri.v[(kk + 1) % 3] == eb || tri.v[(kk + 2) % 3] == eb) {
                                if (tri.v[(kk + 1) % 3] != ea && tri.v[(kk + 2) % 3] != ea) {
                                    adj_tri_eb = tri.adj[kk];
                                }
                            }
                        }
                        if (kk != nk) {
                            if (ntri.v[(kk + 1) % 3] == ea || ntri.v[(kk + 2) % 3] == ea) {
                                if (ntri.v[(kk + 1) % 3] != eb && ntri.v[(kk + 2) % 3] != eb) {
                                    adj_ntri_ea = ntri.adj[kk];
                                }
                            }
                            if (ntri.v[(kk + 1) % 3] == eb || ntri.v[(kk + 2) % 3] == eb) {
                                if (ntri.v[(kk + 1) % 3] != ea && ntri.v[(kk + 2) % 3] != ea) {
                                    adj_ntri_eb = ntri.adj[kk];
                                }
                            }
                        }
                    }

                    // Rebuild tri as (vc, vd, ea) and neighbor as (vc, eb, vd)
                    // Ensure CCW
                    double o_new1 = orient2d(vertices[vc].x, vertices[vc].y,
                                             vertices[vd].x, vertices[vd].y,
                                             vertices[ea].x, vertices[ea].y);
                    if (o_new1 > 0) {
                        tri.v[0] = vc; tri.v[1] = vd; tri.v[2] = ea;
                    } else {
                        tri.v[0] = vc; tri.v[1] = ea; tri.v[2] = vd;
                    }

                    double o_new2 = orient2d(vertices[vc].x, vertices[vc].y,
                                             vertices[eb].x, vertices[eb].y,
                                             vertices[vd].x, vertices[vd].y);
                    if (o_new2 > 0) {
                        ntri.v[0] = vc; ntri.v[1] = eb; ntri.v[2] = vd;
                    } else {
                        ntri.v[0] = vc; ntri.v[1] = vd; ntri.v[2] = eb;
                    }

                    // Reset constrained flags
                    for (int kk = 0; kk < 3; ++kk) {
                        tri.constrained[kk] = false;
                        ntri.constrained[kk] = false;
                    }

                    // Rebuild adjacencies
                    tri.adj[0] = -1; tri.adj[1] = -1; tri.adj[2] = -1;
                    ntri.adj[0] = -1; ntri.adj[1] = -1; ntri.adj[2] = -1;

                    // Link tri and ntri to each other
                    for (int ki2 = 0; ki2 < 3; ++ki2) {
                        int te0 = tri.v[(ki2 + 1) % 3];
                        int te1 = tri.v[(ki2 + 2) % 3];
                        for (int kj2 = 0; kj2 < 3; ++kj2) {
                            int ne0 = ntri.v[(kj2 + 1) % 3];
                            int ne1 = ntri.v[(kj2 + 2) % 3];
                            if ((te0 == ne1 && te1 == ne0) || (te0 == ne0 && te1 == ne1)) {
                                tri.adj[ki2] = neighbor;
                                ntri.adj[kj2] = ti;
                            }
                        }
                    }

                    // For each edge of tri and ntri, find the correct external neighbor
                    for (int ki2 = 0; ki2 < 3; ++ki2) {
                        if (tri.adj[ki2] >= 0) continue; // already linked
                        int te0 = tri.v[(ki2 + 1) % 3];
                        int te1 = tri.v[(ki2 + 2) % 3];
                        // Check all external adjacencies
                        int candidates[] = {adj_tri_ea, adj_tri_eb, adj_ntri_ea, adj_ntri_eb};
                        for (int c : candidates) {
                            if (c < 0 || !triangles[c].alive) continue;
                            for (int ck = 0; ck < 3; ++ck) {
                                int ce0 = triangles[c].v[(ck + 1) % 3];
                                int ce1 = triangles[c].v[(ck + 2) % 3];
                                if ((ce0 == te0 && ce1 == te1) || (ce0 == te1 && ce1 == te0)) {
                                    tri.adj[ki2] = c;
                                    triangles[c].adj[ck] = ti;
                                    goto next_tri_edge;
                                }
                            }
                        }
                        next_tri_edge:;
                    }

                    for (int ki2 = 0; ki2 < 3; ++ki2) {
                        if (ntri.adj[ki2] >= 0) continue;
                        int te0 = ntri.v[(ki2 + 1) % 3];
                        int te1 = ntri.v[(ki2 + 2) % 3];
                        int candidates[] = {adj_tri_ea, adj_tri_eb, adj_ntri_ea, adj_ntri_eb};
                        for (int c : candidates) {
                            if (c < 0 || !triangles[c].alive) continue;
                            for (int ck = 0; ck < 3; ++ck) {
                                int ce0 = triangles[c].v[(ck + 1) % 3];
                                int ce1 = triangles[c].v[(ck + 2) % 3];
                                if ((ce0 == te0 && ce1 == te1) || (ce0 == te1 && ce1 == te0)) {
                                    ntri.adj[ki2] = c;
                                    triangles[c].adj[ck] = neighbor;
                                    goto next_ntri_edge;
                                }
                            }
                        }
                        next_ntri_edge:;
                    }

                    flipped = true;
                    break;
                }
            }
        }
        if (!flipped) break;
    }

    // Final check - mark if edge now exists
    if (edge_exists(v0, v1)) {
        mark_constrained(v0, v1);
    }
}

void Delaunay2D::cleanup() {
    // Remove all triangles connected to super-triangle vertices
    for (auto& tri : triangles) {
        if (!tri.alive) continue;
        for (int k = 0; k < 3; ++k) {
            if (tri.v[k] == super_v[0] || tri.v[k] == super_v[1] || tri.v[k] == super_v[2]) {
                tri.alive = false;
                break;
            }
        }
    }
}

std::vector<std::array<int, 3>> Delaunay2D::get_triangles() const {
    std::vector<std::array<int, 3>> result;
    for (const auto& tri : triangles) {
        if (!tri.alive) continue;
        // Ensure CCW
        double o = orient2d(vertices[tri.v[0]].x, vertices[tri.v[0]].y,
                            vertices[tri.v[1]].x, vertices[tri.v[1]].y,
                            vertices[tri.v[2]].x, vertices[tri.v[2]].y);
        if (o > 0) {
            result.push_back({tri.v[0], tri.v[1], tri.v[2]});
        } else {
            result.push_back({tri.v[0], tri.v[2], tri.v[1]});
        }
    }
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// NurbsTriangulation Implementation
///////////////////////////////////////////////////////////////////////////////////////////

NurbsTriangulation::NurbsTriangulation(const NurbsSurface& surface)
    : m_surface(surface) {}

NurbsTriangulation& NurbsTriangulation::set_max_angle(double degrees) {
    m_max_angle = degrees;
    return *this;
}

NurbsTriangulation& NurbsTriangulation::set_max_edge_length(double length) {
    m_max_edge_length = length;
    return *this;
}

NurbsTriangulation& NurbsTriangulation::set_min_edge_length(double length) {
    m_min_edge_length = length;
    return *this;
}

NurbsTriangulation& NurbsTriangulation::set_max_chord_height(double height) {
    m_max_chord_height = height;
    return *this;
}

NurbsTriangulation& NurbsTriangulation::set_max_iterations(int iterations) {
    m_max_iterations = iterations;
    return *this;
}

double NurbsTriangulation::compute_bbox_diagonal() const {
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

int NurbsTriangulation::find_worst_triangle(const Delaunay2D& dt, double max_edge,
                                             double chord_tol, double angle_rad,
                                             double min_edge) const {
    double worst_score = 0.0;
    int worst_ti = -1;

    for (int ti = 0; ti < static_cast<int>(dt.triangles.size()); ++ti) {
        const Triangle& tri = dt.triangles[ti];
        if (!tri.alive) continue;

        // Skip super-triangle vertices
        bool has_super = false;
        for (int k = 0; k < 3; ++k) {
            if (tri.v[k] == dt.super_v[0] || tri.v[k] == dt.super_v[1] || tri.v[k] == dt.super_v[2]) {
                has_super = true;
                break;
            }
        }
        if (has_super) continue;

        double u0 = dt.vertices[tri.v[0]].x, v0 = dt.vertices[tri.v[0]].y;
        double u1 = dt.vertices[tri.v[1]].x, v1 = dt.vertices[tri.v[1]].y;
        double u2 = dt.vertices[tri.v[2]].x, v2 = dt.vertices[tri.v[2]].y;

        Point p0 = m_surface.point_at(u0, v0);
        Point p1 = m_surface.point_at(u1, v1);
        Point p2 = m_surface.point_at(u2, v2);

        double e01 = p0.distance(p1);
        double e12 = p1.distance(p2);
        double e20 = p2.distance(p0);
        double max_e = std::max({e01, e12, e20});

        if (max_e < min_edge) continue;

        double score = 0.0;

        // Max edge length violation
        if (max_e > max_edge) {
            score = std::max(score, max_e / max_edge);
        }

        // Chord height at centroid
        double uc = (u0 + u1 + u2) / 3.0;
        double vc = (v0 + v1 + v2) / 3.0;
        Point p_surface = m_surface.point_at(uc, vc);
        Point p_linear((p0[0] + p1[0] + p2[0]) / 3.0,
                       (p0[1] + p1[1] + p2[1]) / 3.0,
                       (p0[2] + p1[2] + p2[2]) / 3.0);
        double chord_h = p_surface.distance(p_linear);
        if (chord_h > chord_tol) {
            score = std::max(score, chord_h / chord_tol);
        }

        // Edge midpoint chord heights
        auto check_midpoint = [&](double ua, double va2, double ub, double vb,
                                   const Point& pa, const Point& pb) {
            double um = (ua + ub) * 0.5;
            double vm = (va2 + vb) * 0.5;
            Point p_mid = m_surface.point_at(um, vm);
            Point p_lin((pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5, (pa[2] + pb[2]) * 0.5);
            double h = p_mid.distance(p_lin);
            if (h > chord_tol) {
                score = std::max(score, h / chord_tol);
            }
        };
        check_midpoint(u0, v0, u1, v1, p0, p1);
        check_midpoint(u1, v1, u2, v2, p1, p2);
        check_midpoint(u2, v2, u0, v0, p2, p0);

        // Normal angle deviation
        Vector n0 = m_surface.normal_at(u0, v0);
        Vector n1 = m_surface.normal_at(u1, v1);
        Vector n2 = m_surface.normal_at(u2, v2);

        auto check_angle = [&](const Vector& na, const Vector& nb) {
            double d = na.dot(nb);
            d = std::max(-1.0, std::min(1.0, d));
            double angle = std::acos(d);
            if (angle > angle_rad) {
                score = std::max(score, angle / angle_rad);
            }
        };
        check_angle(n0, n1);
        check_angle(n1, n2);
        check_angle(n2, n0);

        if (score > worst_score) {
            worst_score = score;
            worst_ti = ti;
        }
    }

    return worst_ti;
}

// Point-in-polygon test (ray casting)
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

Mesh NurbsTriangulation::mesh() const {
    double bbox_diag = compute_bbox_diagonal();
    double max_edge = m_max_edge_length > 0 ? m_max_edge_length : bbox_diag / 10.0;
    double min_edge = m_min_edge_length > 0 ? m_min_edge_length : bbox_diag / 1000.0;
    double chord_tol = m_max_chord_height > 0 ? m_max_chord_height : bbox_diag / 500.0;
    double angle_rad = m_max_angle * Tolerance::PI / 180.0;

    // Get UV domain
    auto [u_min, u_max] = m_surface.domain(0);
    auto [v_min, v_max] = m_surface.domain(1);

    bool trimmed = m_surface.is_trimmed();
    std::vector<Vertex2D> trim_poly;

    // Initialize Delaunay2D
    Delaunay2D dt(u_min, v_min, u_max, v_max);

    if (trimmed) {
        // Sample the outer loop to get trim polygon vertices in UV space
        NurbsCurve loop = m_surface.get_outer_loop();
        auto [loop_pts, loop_params] = loop.divide_by_count(50, true);
        for (const auto& pt : loop_pts) {
            trim_poly.push_back({pt[0], pt[1]});
        }

        // Insert trim polygon vertices into Delaunay
        std::vector<int> trim_verts;
        for (const auto& v : trim_poly) {
            trim_verts.push_back(dt.insert(v.x, v.y));
        }

        // Also insert some interior grid points that are inside the trim
        int grid_n = 8;
        double du = (u_max - u_min) / (grid_n + 1);
        double dv = (v_max - v_min) / (grid_n + 1);
        for (int i = 1; i <= grid_n; ++i) {
            for (int j = 1; j <= grid_n; ++j) {
                double gu = u_min + i * du;
                double gv = v_min + j * dv;
                if (point_in_polygon(gu, gv, trim_poly)) {
                    dt.insert(gu, gv);
                }
            }
        }

        // Constrain edges along trim polygon
        for (size_t i = 0; i + 1 < trim_verts.size(); ++i) {
            if (trim_verts[i] >= 0 && trim_verts[i + 1] >= 0) {
                dt.insert_constraint(trim_verts[i], trim_verts[i + 1]);
            }
        }

        dt.cleanup();

        // Remove triangles outside trim polygon
        for (auto& tri : dt.triangles) {
            if (!tri.alive) continue;
            double cu = (dt.vertices[tri.v[0]].x + dt.vertices[tri.v[1]].x + dt.vertices[tri.v[2]].x) / 3.0;
            double cv_val = (dt.vertices[tri.v[0]].y + dt.vertices[tri.v[1]].y + dt.vertices[tri.v[2]].y) / 3.0;
            if (!point_in_polygon(cu, cv_val, trim_poly)) {
                tri.alive = false;
            }
        }
    } else {
        // Untrimmed path: use UV rectangle boundary
        int c0 = dt.insert(u_min, v_min);
        int c1 = dt.insert(u_max, v_min);
        int c2 = dt.insert(u_max, v_max);
        int c3 = dt.insert(u_min, v_max);

        std::vector<double> u_spans = m_surface.get_span_vector(0);
        std::vector<double> v_spans = m_surface.get_span_vector(1);

        std::vector<int> bottom_verts, top_verts;
        bottom_verts.push_back(c0);
        top_verts.push_back(c3);
        for (size_t i = 1; i < u_spans.size() - 1; ++i) {
            bottom_verts.push_back(dt.insert(u_spans[i], v_min));
            top_verts.push_back(dt.insert(u_spans[i], v_max));
        }
        bottom_verts.push_back(c1);
        top_verts.push_back(c2);

        std::vector<int> left_verts, right_verts;
        left_verts.push_back(c0);
        right_verts.push_back(c1);
        for (size_t i = 1; i < v_spans.size() - 1; ++i) {
            left_verts.push_back(dt.insert(u_min, v_spans[i]));
            right_verts.push_back(dt.insert(u_max, v_spans[i]));
        }
        left_verts.push_back(c3);
        right_verts.push_back(c2);

        for (size_t i = 1; i < u_spans.size() - 1; ++i) {
            for (size_t j = 1; j < v_spans.size() - 1; ++j) {
                dt.insert(u_spans[i], v_spans[j]);
            }
        }

        auto constrain_chain = [&](const std::vector<int>& verts) {
            for (size_t i = 0; i + 1 < verts.size(); ++i) {
                if (verts[i] >= 0 && verts[i + 1] >= 0) {
                    dt.insert_constraint(verts[i], verts[i + 1]);
                }
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
            if (cu < u_min - eps || cu > u_max + eps || cv_val < v_min - eps || cv_val > v_max + eps) {
                tri.alive = false;
            }
        }
    }

    // Refinement loop
    for (int iter = 0; iter < m_max_iterations; ++iter) {
        int worst = find_worst_triangle(dt, max_edge, chord_tol, angle_rad, min_edge);
        if (worst < 0) break;

        const Triangle& wtri = dt.triangles[worst];
        double cu, cv_val;
        Delaunay2D::circumcenter(
            dt.vertices[wtri.v[0]].x, dt.vertices[wtri.v[0]].y,
            dt.vertices[wtri.v[1]].x, dt.vertices[wtri.v[1]].y,
            dt.vertices[wtri.v[2]].x, dt.vertices[wtri.v[2]].y,
            cu, cv_val);

        cu = std::max(u_min, std::min(u_max, cu));
        cv_val = std::max(v_min, std::min(v_max, cv_val));

        if (trimmed && !point_in_polygon(cu, cv_val, trim_poly)) continue;

        int new_v = dt.insert(cu, cv_val);
        if (new_v < 0) break;

        // Remove newly created triangles outside boundary
        for (auto& tri : dt.triangles) {
            if (!tri.alive) continue;
            bool has_new = false;
            for (int k = 0; k < 3; ++k) {
                if (tri.v[k] == new_v) { has_new = true; break; }
            }
            if (!has_new) continue;
            double tc_u = (dt.vertices[tri.v[0]].x + dt.vertices[tri.v[1]].x + dt.vertices[tri.v[2]].x) / 3.0;
            double tc_v = (dt.vertices[tri.v[0]].y + dt.vertices[tri.v[1]].y + dt.vertices[tri.v[2]].y) / 3.0;
            if (trimmed) {
                if (!point_in_polygon(tc_u, tc_v, trim_poly)) tri.alive = false;
            } else {
                double eps = 1e-10;
                if (tc_u < u_min - eps || tc_u > u_max + eps || tc_v < v_min - eps || tc_v > v_max + eps)
                    tri.alive = false;
            }
        }
    }

    // Build output Mesh
    Mesh result;
    std::unordered_map<int, size_t> vertex_map;

    auto tris = dt.get_triangles();
    for (const auto& tri : tris) {
        for (int k = 0; k < 3; ++k) {
            int vi = tri[k];
            if (vertex_map.find(vi) == vertex_map.end()) {
                double u = dt.vertices[vi].x;
                double v = dt.vertices[vi].y;
                Point pt = m_surface.point_at(u, v);
                Vector n = m_surface.normal_at(u, v);
                size_t vk = result.add_vertex(pt);
                result.vertex[vk].set_normal(n[0], n[1], n[2]);
                vertex_map[vi] = vk;
            }
        }
        std::vector<size_t> face_verts = {
            vertex_map[tri[0]],
            vertex_map[tri[1]],
            vertex_map[tri[2]]
        };
        result.add_face(face_verts);
    }

    return result;
}

} // namespace session_cpp
