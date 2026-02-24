#include "trimesh_remesh.h"
#include "trimesh_delaunay.h" // FlatMap64
#include <cmath>
#include <algorithm>
#include <vector>
#include <numeric>

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// HalfedgeMesh basics
///////////////////////////////////////////////////////////////////////////////////////////

int HalfedgeMesh::add_vertex(double u, double v) {
    int vi = (int)verts.size();
    verts.push_back({u, v, 0, 0, 0, 0, 0, 0, -1, 0});
    return vi;
}

int HalfedgeMesh::add_face(int va, int vb, int vc) {
    int fi = (int)faces.size();
    int h0 = (int)hedges.size();
    faces.push_back({h0});
    hedges.push_back({h0 + 1, -1, vb, fi}); // h0: va→vb, head=vb
    hedges.push_back({h0 + 2, -1, vc, fi}); // h1: vb→vc, head=vc
    hedges.push_back({h0,     -1, va, fi}); // h2: vc→va, head=va
    if (verts[va].halfedge < 0) verts[va].halfedge = h0 + 2; // head=va
    if (verts[vb].halfedge < 0) verts[vb].halfedge = h0;     // head=vb
    if (verts[vc].halfedge < 0) verts[vc].halfedge = h0 + 1; // head=vc
    return fi;
}

int HalfedgeMesh::valence(int vi) const {
    int he_start = verts[vi].halfedge;
    if (he_start < 0) return 0;
    // Simple linear scan — count halfedges with head=vi in live faces
    int count = 0;
    for (int i = 0; i < (int)hedges.size(); ++i)
        if (hedges[i].vertex == vi && hedges[i].face >= 0 && faces[hedges[i].face].halfedge >= 0)
            ++count;
    return count;
}

double HalfedgeMesh::face_area(int fi) const {
    int h0 = faces[fi].halfedge;
    if (h0 < 0) return 0.0;
    int h1 = hedges[h0].next;
    int h2 = hedges[h1].next;
    int va = hedges[h2].vertex, vb = hedges[h0].vertex, vc = hedges[h1].vertex;
    double e1x = verts[vb].px - verts[va].px;
    double e1y = verts[vb].py - verts[va].py;
    double e1z = verts[vb].pz - verts[va].pz;
    double e2x = verts[vc].px - verts[va].px;
    double e2y = verts[vc].py - verts[va].py;
    double e2z = verts[vc].pz - verts[va].pz;
    double cx = e1y * e2z - e1z * e2y;
    double cy = e1z * e2x - e1x * e2z;
    double cz = e1x * e2y - e1y * e2x;
    return 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
}

double HalfedgeMesh::edge_length_3d(int he) const {
    int prev = hedges[hedges[he].next].next;
    int va = hedges[prev].vertex; // tail
    int vb = hedges[he].vertex;   // head
    double dx = verts[vb].px - verts[va].px;
    double dy = verts[vb].py - verts[va].py;
    double dz = verts[vb].pz - verts[va].pz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

///////////////////////////////////////////////////////////////////////////////////////////
// TrimeshRemesh
///////////////////////////////////////////////////////////////////////////////////////////

TrimeshRemesh::TrimeshRemesh(const NurbsSurface& surface) : m_surface(surface) {}

TrimeshRemesh& TrimeshRemesh::set_max_angle(double degrees) { m_max_angle = degrees; return *this; }
TrimeshRemesh& TrimeshRemesh::set_max_edge_length(double length) { m_max_edge_length = length; return *this; }
TrimeshRemesh& TrimeshRemesh::set_min_edge_length(double length) { m_min_edge_length = length; return *this; }
TrimeshRemesh& TrimeshRemesh::set_max_chord_height(double height) { m_max_chord_height = height; return *this; }
TrimeshRemesh& TrimeshRemesh::set_iterations(int iterations) { m_iterations = iterations; return *this; }

double TrimeshRemesh::compute_bbox_diagonal() const {
    double minx = 1e30, miny = 1e30, minz = 1e30;
    double maxx = -1e30, maxy = -1e30, maxz = -1e30;
    for (int i = 0; i < m_surface.cv_count(0); ++i)
        for (int j = 0; j < m_surface.cv_count(1); ++j) {
            Point p = m_surface.get_cv(i, j);
            minx = std::min(minx, p[0]); miny = std::min(miny, p[1]); minz = std::min(minz, p[2]);
            maxx = std::max(maxx, p[0]); maxy = std::max(maxy, p[1]); maxz = std::max(maxz, p[2]);
        }
    double dx = maxx - minx, dy = maxy - miny, dz = maxz - minz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

///////////////////////////////////////////////////////////////////////////////////////////
// UV wrapping helpers
///////////////////////////////////////////////////////////////////////////////////////////

static double wrap_mid(double a, double b, double lo, double hi, bool closed) {
    double width = hi - lo;
    if (closed && std::abs(b - a) > width * 0.5)
        b += (b < a) ? width : -width;
    double m = (a + b) * 0.5;
    if (closed) {
        if (m < lo) m += width;
        if (m >= hi) m -= width;
    }
    return m;
}

static double unwrap(double neighbor, double center, double lo, double hi, bool closed) {
    if (!closed) return neighbor;
    double width = hi - lo;
    double d = neighbor - center;
    if (d > width * 0.5) return neighbor - width;
    if (d < -width * 0.5) return neighbor + width;
    return neighbor;
}

static uint64_t edge_key(int a, int b) {
    return ((uint64_t)(uint32_t)std::min(a, b) << 32) | (uint64_t)(uint32_t)std::max(a, b);
}

///////////////////////////////////////////////////////////////////////////////////////////
// build_initial_mesh — curvature-adaptive grid reused from trimesh_grid
///////////////////////////////////////////////////////////////////////////////////////////

void TrimeshRemesh::build_initial_mesh(HalfedgeMesh& hm) const {
    std::vector<double> usp = m_surface.get_span_vector(0);
    std::vector<double> vsp = m_surface.get_span_vector(1);
    int ns_u = (int)usp.size() - 1, ns_v = (int)vsp.size() - 1;
    int deg_u = m_surface.degree(0), deg_v = m_surface.degree(1);
    double bbox_diag = compute_bbox_diagonal();

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
                double max_a = 0.0;
                for (int si = 0; si < n_other; ++si) {
                    double s = s_positions[si];
                    double prev_nx = 0, prev_ny = 0, prev_nz = 0;
                    double total_angle = 0.0;
                    for (int k = 0; k <= 4; ++k) {
                        double t = t0 + k * (t1 - t0) / 4.0;
                        Vector nrm = (dir == 0) ? m_surface.normal_at(t, s) : m_surface.normal_at(s, t);
                        if (k > 0) {
                            double dot = prev_nx*nrm[0] + prev_ny*nrm[1] + prev_nz*nrm[2];
                            dot = std::max(-1.0, std::min(1.0, dot));
                            total_angle += std::acos(dot) * 180.0 / Tolerance::PI;
                        }
                        prev_nx = nrm[0]; prev_ny = nrm[1]; prev_nz = nrm[2];
                    }
                    if (total_angle > max_a) max_a = total_angle;
                }
                subs[i] = std::max(1, std::min((int)std::ceil(max_a / m_max_angle), 24));
            }
            {
                double ct = (m_max_chord_height > 0) ? m_max_chord_height : bbox_diag * 0.005;
                double max_dev = 0.0;
                int nc = std::min(n_other, 3);
                for (int ci = 0; ci <= nc; ++ci) {
                    double s = osp.front() + ci * (osp.back() - osp.front()) / std::max(nc, 1);
                    double px0, py0, pz0, px1, py1, pz1;
                    if (dir == 0) { m_surface.point_at(t0, s, px0, py0, pz0); m_surface.point_at(t1, s, px1, py1, pz1); }
                    else          { m_surface.point_at(s, t0, px0, py0, pz0); m_surface.point_at(s, t1, px1, py1, pz1); }
                    for (int k = 1; k <= 3; ++k) {
                        double frac = k / 4.0;
                        double tm = t0 + frac * (t1 - t0);
                        double pmx, pmy, pmz;
                        if (dir == 0) m_surface.point_at(tm, s, pmx, pmy, pmz);
                        else          m_surface.point_at(s, tm, pmx, pmy, pmz);
                        double lx = px0 + frac * (px1 - px0), ly = py0 + frac * (py1 - py0), lz = pz0 + frac * (pz1 - pz0);
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
                if (dir == 0) { m_surface.point_at(t0, s_mid, px0, py0, pz0); m_surface.point_at(t1, s_mid, px1, py1, pz1); }
                else          { m_surface.point_at(s_mid, t0, px0, py0, pz0); m_surface.point_at(s_mid, t1, px1, py1, pz1); }
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
            int ns = std::max(total_u, 10);
            for (int i = 1; i <= ns; ++i) {
                double u = usp.front() + i * (usp.back() - usp.front()) / ns;
                double px1, py1, pz1;
                m_surface.point_at(u, v_mid, px1, py1, pz1);
                u_len += std::sqrt((px1-px0)*(px1-px0)+(py1-py0)*(py1-py0)+(pz1-pz0)*(pz1-pz0));
                px0 = px1; py0 = py1; pz0 = pz1;
            }
        }
        {
            double px0, py0, pz0;
            m_surface.point_at(u_mid, vsp.front(), px0, py0, pz0);
            int ns = std::max(total_v, 10);
            for (int i = 1; i <= ns; ++i) {
                double v = vsp.front() + i * (vsp.back() - vsp.front()) / ns;
                double px1, py1, pz1;
                m_surface.point_at(u_mid, v, px1, py1, pz1);
                v_len += std::sqrt((px1-px0)*(px1-px0)+(py1-py0)*(py1-py0)+(pz1-pz0)*(pz1-pz0));
                px0 = px1; py0 = py1; pz0 = pz1;
            }
        }
        if (u_len > 1e-14 && v_len > 1e-14 && total_u > 0 && total_v > 0) {
            double spacing_u = u_len / total_u, spacing_v = v_len / total_v;
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
    if (deg_u == 1 && deg_v == 1) {
        double chord_tol = (bbox_diag > 0) ? bbox_diag * 0.005 : 1e-6;
        double max_twist = 0.0;
        for (int i = 0; i < ns_u; ++i)
            for (int j = 0; j < ns_v; ++j) {
                double u0 = usp[i], u1 = usp[i+1], v0 = vsp[j], v1 = vsp[j+1];
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
        if (max_twist > chord_tol) {
            int twist_subs = std::max(4, std::min((int)std::ceil(2.0 * std::sqrt(max_twist / chord_tol)), 24));
            for (int& s : u_subs) s = std::max(s, twist_subs);
            for (int& s : v_subs) s = std::max(s, twist_subs);
        }
    }

    // Build parameter arrays
    std::vector<double> us, vs;
    for (int i = 0; i < ns_u; ++i)
        for (int s = 0; s < u_subs[i]; ++s)
            us.push_back(usp[i] + s * (usp[i+1] - usp[i]) / u_subs[i]);
    us.push_back(usp.back());
    for (int i = 0; i < ns_v; ++i)
        for (int s = 0; s < v_subs[i]; ++s)
            vs.push_back(vsp[i] + s * (vsp[i+1] - vsp[i]) / v_subs[i]);
    vs.push_back(vsp.back());

    bool closed_u = m_surface.is_closed(0);
    bool closed_v = m_surface.is_closed(1);

    auto fix_closed_gap = [](std::vector<double>& params, const std::vector<double>& spans, bool closed) {
        if (!closed || params.size() < 3) return;
        params.pop_back();
        double domain_end = spans.back();
        double wrap_gap = domain_end - params.back();
        double max_gap = 0;
        for (size_t i = 1; i < params.size(); i++)
            max_gap = std::max(max_gap, params[i] - params[i - 1]);
        if (max_gap > 0 && wrap_gap > max_gap * 1.5) {
            int extra = (int)std::ceil(wrap_gap / max_gap) - 1;
            double step = wrap_gap / (extra + 1);
            for (int e = 1; e <= extra; e++)
                params.push_back(params.back() + step);
        }
    };
    fix_closed_gap(us, usp, closed_u);
    fix_closed_gap(vs, vsp, closed_v);

    int nu = (int)us.size(), nv = (int)vs.size();
    bool sing_v0 = m_surface.is_singular(0);
    bool sing_v1 = m_surface.is_singular(2);
    int j_start = sing_v0 ? 1 : 0;
    int j_end = sing_v1 ? nv - 1 : nv;
    int nv_grid = j_end - j_start;
    int nu_eff = closed_u ? nu : nu; // all columns in param array (closed already has last removed)

    // Create vertices
    int south_pole = -1, north_pole = -1;
    if (sing_v0) {
        south_pole = hm.add_vertex(us[0], vs[0]);
        hm.verts[south_pole].flags |= RVertex::POLE;
    }
    if (sing_v1) {
        north_pole = hm.add_vertex(us[0], vs[nv - 1]);
        hm.verts[north_pole].flags |= RVertex::POLE;
    }

    int grid_base = (int)hm.verts.size();
    for (int i = 0; i < nu; ++i)
        for (int j = j_start; j < j_end; ++j)
            hm.add_vertex(us[i], vs[j]);

    auto grid_idx = [&](int i, int j) -> int {
        return grid_base + i * nv_grid + (j - j_start);
    };

    // Mark boundary vertices for open surfaces
    if (!closed_u) {
        for (int j = j_start; j < j_end; ++j) {
            hm.verts[grid_idx(0, j)].flags |= RVertex::BOUNDARY;
            hm.verts[grid_idx(nu - 1, j)].flags |= RVertex::BOUNDARY;
        }
    }
    if (!closed_v && !sing_v0) {
        for (int i = 0; i < nu; ++i)
            hm.verts[grid_idx(i, j_start)].flags |= RVertex::BOUNDARY;
    }
    if (!closed_v && !sing_v1) {
        for (int i = 0; i < nu; ++i)
            hm.verts[grid_idx(i, j_end - 1)].flags |= RVertex::BOUNDARY;
    }

    // Triangulate grid with alternating diagonals
    int nu_faces = closed_u ? nu : nu - 1;
    int nv_interior = nv_grid - 1;
    if (closed_v && !sing_v0 && !sing_v1) nv_interior = nv_grid;

    // Helper: wrap indices for closed directions
    auto wrap_i = [&](int i) -> int { return closed_u ? (i % nu) : i; };
    auto wrap_j = [&](int jj) -> int {
        if (closed_v && !sing_v0 && !sing_v1)
            return (jj % nv_grid) + j_start;
        return jj + j_start + 1; // next row
    };

    // South pole fan
    if (sing_v0) {
        for (int i = 0; i < nu_faces; ++i)
            hm.add_face(south_pole, grid_idx(wrap_i(i + 1), j_start), grid_idx(i, j_start));
    }

    // Interior quads → 2 triangles each
    for (int i = 0; i < nu_faces; ++i)
        for (int jj = 0; jj < nv_interior; ++jj) {
            int j = jj + j_start;
            int i1 = wrap_i(i + 1);
            int j1 = wrap_j(jj);
            int v00 = grid_idx(i, j), v10 = grid_idx(i1, j);
            int v01 = grid_idx(i, j1), v11 = grid_idx(i1, j1);
            if ((i + jj) % 2 == 0) {
                hm.add_face(v00, v10, v11);
                hm.add_face(v00, v11, v01);
            } else {
                hm.add_face(v00, v10, v01);
                hm.add_face(v10, v11, v01);
            }
        }

    // North pole fan
    if (sing_v1) {
        int j_last = j_end - 1;
        for (int i = 0; i < nu_faces; ++i)
            hm.add_face(grid_idx(i, j_last), grid_idx(wrap_i(i + 1), j_last), north_pole);
    }

    // Pair twins using edge map
    FlatMap64<int> emap;
    emap.reserve(hm.hedges.size());
    for (int hi = 0; hi < (int)hm.hedges.size(); ++hi) {
        int h_tail = hm.hedges[hm.hedges[hi].next].next; // prev in triangle
        int va = hm.hedges[h_tail].vertex; // tail of hi
        int vb = hm.hedges[hi].vertex;     // head of hi
        auto key = edge_key(va, vb);
        int* existing = emap.find(key);
        if (existing) {
            int other = *existing;
            hm.hedges[hi].twin = other;
            hm.hedges[other].twin = hi;
            emap.erase(key);
        } else {
            emap[key] = hi;
        }
    }

    // Boundary halfedge loops for open edges (unpaired halfedges)
    // For open surfaces, create boundary halfedges (face = -1) to complete the twin pairing
    // Skip this — boundary edges simply have twin = -1

    // Evaluate 3D positions and normals
    for (auto& v : hm.verts) {
        Point pt = m_surface.point_at(v.u, v.v);
        Vector nm = m_surface.normal_at(v.u, v.v);
        v.px = pt[0]; v.py = pt[1]; v.pz = pt[2];
        v.nx = nm[0]; v.ny = nm[1]; v.nz = nm[2];
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// split_long_edges — split edge va→vb at midpoint vm, 2 faces → 4 faces
// Before: faceA=(va,vb,vc) faceC=(vb,va,vd)
// After:  faceA=(va,vm,vc) faceB=(vm,vb,vc) faceC=(bv,vm,vd) faceD=(vm,va,vd)
///////////////////////////////////////////////////////////////////////////////////////////

void TrimeshRemesh::split_long_edges(HalfedgeMesh& hm, double max_len) const {
    auto [u_min, u_max] = m_surface.domain(0);
    auto [v_min, v_max] = m_surface.domain(1);
    bool closed_u = m_surface.is_closed(0);
    bool closed_v = m_surface.is_closed(1);

    std::vector<std::pair<double, int>> edges;
    for (int hi = 0; hi < (int)hm.hedges.size(); ++hi) {
        if (hm.hedges[hi].face < 0) continue;
        if (hm.faces[hm.hedges[hi].face].halfedge < 0) continue;
        int tw = hm.hedges[hi].twin;
        if (tw >= 0 && tw < hi) continue;
        double len = hm.edge_length_3d(hi);
        if (len > max_len)
            edges.push_back({len, hi});
    }
    std::sort(edges.begin(), edges.end(), [](auto& a, auto& b) { return a.first > b.first; });

    for (auto& [len, he0] : edges) {
        if (hm.hedges[he0].face < 0) continue;
        int fi0 = hm.hedges[he0].face;
        if (hm.faces[fi0].halfedge < 0) continue;

        int tw0 = hm.hedges[he0].twin;
        bool has_twin = (tw0 >= 0 && hm.hedges[tw0].face >= 0 && hm.faces[hm.hedges[tw0].face].halfedge >= 0);

        int h0_next = hm.hedges[he0].next;
        int h0_prev = hm.hedges[h0_next].next;
        int va = hm.hedges[h0_prev].vertex;
        int vb = hm.hedges[he0].vertex;
        int vc = hm.hedges[h0_next].vertex;

        if (hm.verts[va].flags & RVertex::POLE) continue;
        if (hm.verts[vb].flags & RVertex::POLE) continue;

        double um = wrap_mid(hm.verts[va].u, hm.verts[vb].u, u_min, u_max, closed_u);
        double vm = wrap_mid(hm.verts[va].v, hm.verts[vb].v, v_min, v_max, closed_v);
        int M = hm.add_vertex(um, vm);
        if ((hm.verts[va].flags & RVertex::BOUNDARY) && (hm.verts[vb].flags & RVertex::BOUNDARY))
            hm.verts[M].flags |= RVertex::BOUNDARY;
        Point pt = m_surface.point_at(um, vm);
        Vector nm = m_surface.normal_at(um, vm);
        hm.verts[M].px = pt[0]; hm.verts[M].py = pt[1]; hm.verts[M].pz = pt[2];
        hm.verts[M].nx = nm[0]; hm.verts[M].ny = nm[1]; hm.verts[M].nz = nm[2];

        // faceA (reuse fi0): he0(va→M), hA(M→vc), h0_prev(vc→va)
        hm.hedges[he0].vertex = M;
        int hA = (int)hm.hedges.size();
        hm.hedges.push_back({h0_prev, -1, vc, fi0});
        hm.hedges[he0].next = hA;

        // faceB (new): hB(M→vb), h0_next(vb→vc), hC(vc→M)
        int fB = (int)hm.faces.size();
        int hB = (int)hm.hedges.size();
        hm.faces.push_back({hB});
        hm.hedges.push_back({h0_next, -1, vb, fB});
        int hC = (int)hm.hedges.size();
        hm.hedges.push_back({hB, -1, M, fB});
        hm.hedges[h0_next].next = hC;
        hm.hedges[h0_next].face = fB;

        // Twin: hA(M→vc) ↔ hC(vc→M)
        hm.hedges[hA].twin = hC;
        hm.hedges[hC].twin = hA;

        hm.verts[M].halfedge = hC;
        if (hm.verts[vb].halfedge == he0)
            hm.verts[vb].halfedge = hB;

        if (has_twin) {
            int tw_next = hm.hedges[tw0].next;
            int tw_prev = hm.hedges[tw_next].next;
            int vd = hm.hedges[tw_next].vertex;
            int fi1 = hm.hedges[tw0].face;

            // faceC (reuse fi1): tw0(vb→M), hD(M→vd), tw_prev(vd→vb)
            hm.hedges[tw0].vertex = M;
            int hD = (int)hm.hedges.size();
            hm.hedges.push_back({tw_prev, -1, vd, fi1});
            hm.hedges[tw0].next = hD;

            // faceD (new): hE(M→va), tw_next(va→vd), hF(vd→M)
            int fD = (int)hm.faces.size();
            int hE = (int)hm.hedges.size();
            hm.faces.push_back({hE});
            hm.hedges.push_back({tw_next, -1, va, fD});
            int hF = (int)hm.hedges.size();
            hm.hedges.push_back({hE, -1, M, fD});
            hm.hedges[tw_next].next = hF;
            hm.hedges[tw_next].face = fD;

            // Twin: hD(M→vd) ↔ hF(vd→M)
            hm.hedges[hD].twin = hF;
            hm.hedges[hF].twin = hD;
            // Twin: he0(va→M) ↔ hE(M→va)
            hm.hedges[he0].twin = hE;
            hm.hedges[hE].twin = he0;
            // Twin: tw0(vb→M) ↔ hB(M→vb)
            hm.hedges[tw0].twin = hB;
            hm.hedges[hB].twin = tw0;

            if (hm.verts[va].halfedge == tw0)
                hm.verts[va].halfedge = hE;
        } else {
            hm.hedges[he0].twin = -1;
            hm.hedges[hB].twin = -1;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// collapse_short_edges — merge two vertices, remove 2 adjacent faces
///////////////////////////////////////////////////////////////////////////////////////////

void TrimeshRemesh::collapse_short_edges(HalfedgeMesh& hm, double min_len) const {
    auto [u_min, u_max] = m_surface.domain(0);
    auto [v_min, v_max] = m_surface.domain(1);
    bool closed_u = m_surface.is_closed(0);
    bool closed_v = m_surface.is_closed(1);

    // Collect edges sorted by length (shortest first)
    std::vector<std::pair<double, int>> edges;
    for (int hi = 0; hi < (int)hm.hedges.size(); ++hi) {
        if (hm.hedges[hi].face < 0) continue;
        if (hm.faces[hm.hedges[hi].face].halfedge < 0) continue;
        int tw = hm.hedges[hi].twin;
        if (tw >= 0 && tw < hi) continue;
        double len = hm.edge_length_3d(hi);
        if (len < min_len)
            edges.push_back({len, hi});
    }
    std::sort(edges.begin(), edges.end());

    for (auto& [len, he0] : edges) {
        if (hm.hedges[he0].face < 0) continue;
        int fi0 = hm.hedges[he0].face;
        if (hm.faces[fi0].halfedge < 0) continue;

        int h0_next = hm.hedges[he0].next;
        int h0_prev = hm.hedges[h0_next].next;
        int va = hm.hedges[h0_prev].vertex;
        int vb = hm.hedges[he0].vertex;
        int vc = hm.hedges[h0_next].vertex;

        if (hm.verts[va].flags & RVertex::DELETED) continue;
        if (hm.verts[vb].flags & RVertex::DELETED) continue;
        if ((hm.verts[va].flags & RVertex::POLE) && (hm.verts[vb].flags & RVertex::POLE)) continue;

        int tw0 = hm.hedges[he0].twin;
        bool has_twin = (tw0 >= 0 && hm.hedges[tw0].face >= 0 && hm.faces[hm.hedges[tw0].face].halfedge >= 0);

        int vd = -1;
        int tw_next = -1, tw_prev = -1;
        if (has_twin) {
            tw_next = hm.hedges[tw0].next;
            tw_prev = hm.hedges[tw_next].next;
            vd = hm.hedges[tw_next].vertex;
        }

        // Don't collapse two boundary vertices (would shrink boundary)
        if ((hm.verts[va].flags & RVertex::BOUNDARY) && (hm.verts[vb].flags & RVertex::BOUNDARY))
            continue;

        // Link condition: common neighbors of va and vb must be exactly {vc, vd}
        auto get_neighbors = [&](int vi) -> std::vector<int> {
            std::vector<int> nbrs;
            for (int hi = 0; hi < (int)hm.hedges.size(); ++hi) {
                if (hm.hedges[hi].vertex != vi) continue;
                int f = hm.hedges[hi].face;
                if (f < 0 || hm.faces[f].halfedge < 0) continue;
                int fh = hi;
                for (int k = 0; k < 3; ++k) {
                    int fv = hm.hedges[fh].vertex;
                    if (fv != vi) {
                        bool found = false;
                        for (int n : nbrs) if (n == fv) { found = true; break; }
                        if (!found) nbrs.push_back(fv);
                    }
                    fh = hm.hedges[fh].next;
                }
            }
            return nbrs;
        };

        auto nbrs_a = get_neighbors(va);
        auto nbrs_b = get_neighbors(vb);
        int common_count = 0;
        for (int na : nbrs_a)
            for (int nb : nbrs_b)
                if (na == nb) { ++common_count; break; }

        int expected = has_twin ? 2 : 1;
        if (common_count != expected) continue;

        // Choose survivor: POLE > BOUNDARY > va
        int survivor = va, victim = vb;
        if (hm.verts[vb].flags & RVertex::POLE) { survivor = vb; victim = va; }
        else if ((hm.verts[vb].flags & RVertex::BOUNDARY) && !(hm.verts[va].flags & RVertex::BOUNDARY))
            { survivor = vb; victim = va; }

        // Update survivor UV (midpoint unless pole/boundary)
        if (!(hm.verts[survivor].flags & (RVertex::POLE | RVertex::BOUNDARY))) {
            hm.verts[survivor].u = wrap_mid(hm.verts[va].u, hm.verts[vb].u, u_min, u_max, closed_u);
            hm.verts[survivor].v = wrap_mid(hm.verts[va].v, hm.verts[vb].v, v_min, v_max, closed_v);
        }

        // Re-point all halfedges with head=victim to head=survivor
        for (int hi = 0; hi < (int)hm.hedges.size(); ++hi)
            if (hm.hedges[hi].vertex == victim)
                hm.hedges[hi].vertex = survivor;

        // Delete the two faces adjacent to the collapsed edge
        // Face0
        {
            hm.faces[fi0].halfedge = -1;
            // Re-twin the outer edges of face0
            // Edges: he0(va→vb), h0_next(vb→vc), h0_prev(vc→va)
            // After vertex redirect, he0(X→survivor), h0_next(survivor→vc), h0_prev(vc→survivor)
            // We need to twin the outer partners of h0_next and h0_prev
            int tw_h0_next = hm.hedges[h0_next].twin;
            int tw_h0_prev = hm.hedges[h0_prev].twin;
            if (tw_h0_next >= 0 && tw_h0_prev >= 0) {
                hm.hedges[tw_h0_next].twin = tw_h0_prev;
                hm.hedges[tw_h0_prev].twin = tw_h0_next;
            } else if (tw_h0_next >= 0) {
                hm.hedges[tw_h0_next].twin = -1;
            } else if (tw_h0_prev >= 0) {
                hm.hedges[tw_h0_prev].twin = -1;
            }
            hm.hedges[he0].face = -1;
            hm.hedges[h0_next].face = -1;
            hm.hedges[h0_prev].face = -1;
        }

        // Face1 (twin side)
        if (has_twin) {
            int fi1 = hm.hedges[tw0].face;
            hm.faces[fi1].halfedge = -1;
            int tw_tw_next = hm.hedges[tw_next].twin;
            int tw_tw_prev = hm.hedges[tw_prev].twin;
            if (tw_tw_next >= 0 && tw_tw_prev >= 0) {
                hm.hedges[tw_tw_next].twin = tw_tw_prev;
                hm.hedges[tw_tw_prev].twin = tw_tw_next;
            } else if (tw_tw_next >= 0) {
                hm.hedges[tw_tw_next].twin = -1;
            } else if (tw_tw_prev >= 0) {
                hm.hedges[tw_tw_prev].twin = -1;
            }
            hm.hedges[tw0].face = -1;
            hm.hedges[tw_next].face = -1;
            hm.hedges[tw_prev].face = -1;
        }

        // Mark victim as deleted
        hm.verts[victim].flags |= RVertex::DELETED;
        hm.verts[victim].halfedge = -1;

        // Fix survivor's halfedge ref
        hm.verts[survivor].halfedge = -1;
        for (int hi = 0; hi < (int)hm.hedges.size(); ++hi) {
            if (hm.hedges[hi].vertex == survivor && hm.hedges[hi].face >= 0 &&
                hm.faces[hm.hedges[hi].face].halfedge >= 0) {
                hm.verts[survivor].halfedge = hi;
                break;
            }
        }

        // Re-evaluate survivor position
        if (!(hm.verts[survivor].flags & RVertex::POLE)) {
            Point pt = m_surface.point_at(hm.verts[survivor].u, hm.verts[survivor].v);
            Vector nm = m_surface.normal_at(hm.verts[survivor].u, hm.verts[survivor].v);
            hm.verts[survivor].px = pt[0]; hm.verts[survivor].py = pt[1]; hm.verts[survivor].pz = pt[2];
            hm.verts[survivor].nx = nm[0]; hm.verts[survivor].ny = nm[1]; hm.verts[survivor].nz = nm[2];
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// flip_edges
///////////////////////////////////////////////////////////////////////////////////////////

void TrimeshRemesh::flip_edges(HalfedgeMesh& hm) const {
    for (int hi = 0; hi < (int)hm.hedges.size(); ++hi) {
        int tw = hm.hedges[hi].twin;
        if (tw < 0 || tw < hi) continue; // process each edge once, skip boundary
        int fi0 = hm.hedges[hi].face;
        int fi1 = hm.hedges[tw].face;
        if (fi0 < 0 || fi1 < 0) continue;
        if (hm.faces[fi0].halfedge < 0 || hm.faces[fi1].halfedge < 0) continue;

        // hi: va→vb, face0=(va,vb,vc)
        int h0 = hi;
        int h1 = hm.hedges[h0].next;
        int h2 = hm.hedges[h1].next;
        int va = hm.hedges[h2].vertex;
        int vb = hm.hedges[h0].vertex;
        int vc = hm.hedges[h1].vertex;

        // tw: vb→va, face1=(vb,va,vd)
        int t0 = tw;
        int t1 = hm.hedges[t0].next;
        int t2 = hm.hedges[t1].next;
        int vd = hm.hedges[t1].vertex;

        // Skip if any pole vertex
        if (hm.verts[va].flags & RVertex::POLE) continue;
        if (hm.verts[vb].flags & RVertex::POLE) continue;
        if (hm.verts[vc].flags & RVertex::POLE) continue;
        if (hm.verts[vd].flags & RVertex::POLE) continue;

        // Valence criterion
        auto target_val = [&](int v) -> int {
            return (hm.verts[v].flags & RVertex::BOUNDARY) ? 4 : 6;
        };
        int val_a = hm.valence(va), val_b = hm.valence(vb);
        int val_c = hm.valence(vc), val_d = hm.valence(vd);

        auto sq = [](int x) { return x * x; };
        int dev_before = sq(val_a - target_val(va)) + sq(val_b - target_val(vb))
                       + sq(val_c - target_val(vc)) + sq(val_d - target_val(vd));
        int dev_after  = sq(val_a - 1 - target_val(va)) + sq(val_b - 1 - target_val(vb))
                       + sq(val_c + 1 - target_val(vc)) + sq(val_d + 1 - target_val(vd));

        if (dev_after >= dev_before) continue;

        // Flip: edge va-vb → edge vc-vd
        // face0 becomes (vc, vd, vb), face1 becomes (vd, vc, va)
        hm.hedges[h0].vertex = vd; // was vb, now vc→vd
        hm.hedges[h0].next = h2;
        hm.hedges[h1].vertex = vb; // vd→vb
        hm.hedges[h1].next = h0;
        hm.hedges[h2].next = h1;   // vb→vc→... wait

        // Let me be more careful:
        // face0: h0(→vd), h2(→vb, was →va), h1(→vc)
        // Actually let me just reassign all 6 halfedges cleanly.

        // face0 = (vc, vd, vb): h1(vc→vd), h0..wait
        // Just rewire:
        // face0: halfedges h0, h1, h2 → (vc→vd, vd→vb, vb→vc)
        hm.hedges[h0].vertex = vd; hm.hedges[h0].next = t2; hm.hedges[h0].face = fi0;
        hm.hedges[t2].vertex = vb; hm.hedges[t2].next = h1; hm.hedges[t2].face = fi0;
        hm.hedges[h1].vertex = vc; hm.hedges[h1].next = h0; hm.hedges[h1].face = fi0;
        hm.faces[fi0].halfedge = h0;

        // face1: halfedges t0, t1, t2 → (vd→vc, vc→va, va→vd)
        hm.hedges[t0].vertex = vc; hm.hedges[t0].next = h2; hm.hedges[t0].face = fi1;
        hm.hedges[h2].vertex = va; hm.hedges[h2].next = t1; hm.hedges[h2].face = fi1;
        hm.hedges[t1].vertex = vd; hm.hedges[t1].next = t0; hm.hedges[t1].face = fi1;
        hm.faces[fi1].halfedge = t0;

        // Update vertex halfedge refs (va, vb might have pointed to deleted halfedges)
        hm.verts[va].halfedge = h2;  // head=va
        hm.verts[vb].halfedge = t2;  // head=vb
        hm.verts[vc].halfedge = h1;  // head=vc
        hm.verts[vd].halfedge = t1;  // head=vd
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// smooth_vertices
///////////////////////////////////////////////////////////////////////////////////////////

void TrimeshRemesh::smooth_vertices(HalfedgeMesh& hm, double strength) const {
    auto [u_min, u_max] = m_surface.domain(0);
    auto [v_min, v_max] = m_surface.domain(1);
    bool closed_u = m_surface.is_closed(0);
    bool closed_v = m_surface.is_closed(1);
    double u_width = u_max - u_min;
    double v_width = v_max - v_min;

    int nv = (int)hm.verts.size();
    std::vector<double> new_u(nv), new_v(nv);
    for (int i = 0; i < nv; ++i) {
        new_u[i] = hm.verts[i].u;
        new_v[i] = hm.verts[i].v;
    }

    for (int vi = 0; vi < nv; ++vi) {
        if (hm.verts[vi].flags & (RVertex::POLE | RVertex::BOUNDARY | RVertex::DELETED)) continue;
        if (hm.verts[vi].halfedge < 0) continue;

        double sum_u = 0, sum_v = 0, sum_w = 0;
        double cu = hm.verts[vi].u, cv = hm.verts[vi].v;

        // Walk around fan
        int he_start = hm.verts[vi].halfedge;
        int he = he_start;
        int safety = 0;
        bool hit_boundary = false;
        do {
            // he has head = vi. The face contains vi and two neighbors.
            int f = hm.hedges[he].face;
            if (f >= 0 && hm.faces[f].halfedge >= 0) {
                // Find neighbor vertex (the tail of he, which is next(next(he))->vertex)
                int prev = hm.hedges[hm.hedges[he].next].next;
                int nb = hm.hedges[prev].vertex; // tail of he
                double area = hm.face_area(f);

                // Also check the other adjacent face for edge weight
                int tw_he = hm.hedges[prev].twin; // twin of prev
                double w = std::sqrt(area);
                if (tw_he >= 0 && hm.hedges[tw_he].face >= 0 && hm.faces[hm.hedges[tw_he].face].halfedge >= 0)
                    w = std::sqrt(area + hm.face_area(hm.hedges[tw_he].face));

                double nu = unwrap(hm.verts[nb].u, cu, u_min, u_max, closed_u);
                double nv_val = unwrap(hm.verts[nb].v, cv, v_min, v_max, closed_v);
                sum_u += nu * w;
                sum_v += nv_val * w;
                sum_w += w;
            }

            int prev = hm.hedges[hm.hedges[he].next].next;
            int tw = hm.hedges[prev].twin;
            if (tw < 0) { hit_boundary = true; break; }
            he = tw;
            if (++safety > 100) break;
        } while (he != he_start);

        // Walk other direction if hit boundary
        if (hit_boundary && he_start >= 0) {
            he = he_start;
            safety = 0;
            while (true) {
                int tw = hm.hedges[he].twin;
                if (tw < 0) break;
                he = hm.hedges[tw].next;
                int f = hm.hedges[he].face;
                if (f >= 0 && hm.faces[f].halfedge >= 0) {
                    int next_he = hm.hedges[he].next;
                    int nb = hm.hedges[next_he].vertex;
                    double area = hm.face_area(f);
                    double w = std::sqrt(area);
                    double nu = unwrap(hm.verts[nb].u, cu, u_min, u_max, closed_u);
                    double nv_val = unwrap(hm.verts[nb].v, cv, v_min, v_max, closed_v);
                    sum_u += nu * w;
                    sum_v += nv_val * w;
                    sum_w += w;
                }
                if (++safety > 100) break;
            }
        }

        if (sum_w > 1e-30) {
            double target_u = sum_u / sum_w;
            double target_v = sum_v / sum_w;
            new_u[vi] = cu + (target_u - cu) * strength;
            new_v[vi] = cv + (target_v - cv) * strength;
            // Wrap back
            if (closed_u) {
                if (new_u[vi] < u_min) new_u[vi] += u_width;
                if (new_u[vi] >= u_max) new_u[vi] -= u_width;
            }
            if (closed_v) {
                if (new_v[vi] < v_min) new_v[vi] += v_width;
                if (new_v[vi] >= v_max) new_v[vi] -= v_width;
            }
        }
    }

    for (int vi = 0; vi < nv; ++vi) {
        hm.verts[vi].u = new_u[vi];
        hm.verts[vi].v = new_v[vi];
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// project_to_surface
///////////////////////////////////////////////////////////////////////////////////////////

void TrimeshRemesh::project_to_surface(HalfedgeMesh& hm) const {
    for (auto& v : hm.verts) {
        if (v.flags & RVertex::DELETED) continue;
        Point pt = m_surface.point_at(v.u, v.v);
        Vector nm = m_surface.normal_at(v.u, v.v);
        v.px = pt[0]; v.py = pt[1]; v.pz = pt[2];
        v.nx = nm[0]; v.ny = nm[1]; v.nz = nm[2];
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// compact
///////////////////////////////////////////////////////////////////////////////////////////

void TrimeshRemesh::compact(HalfedgeMesh& hm) const {
    // Remap vertices
    int nv = (int)hm.verts.size();
    std::vector<int> v_map(nv, -1);
    int new_nv = 0;
    for (int i = 0; i < nv; ++i) {
        if (hm.verts[i].flags & RVertex::DELETED) continue;
        if (hm.verts[i].halfedge < 0) { hm.verts[i].flags |= RVertex::DELETED; continue; }
        v_map[i] = new_nv;
        if (new_nv != i) hm.verts[new_nv] = hm.verts[i];
        ++new_nv;
    }
    hm.verts.resize(new_nv);

    // Remap faces
    int nf = (int)hm.faces.size();
    std::vector<int> f_map(nf, -1);
    int new_nf = 0;
    for (int i = 0; i < nf; ++i) {
        if (hm.faces[i].halfedge < 0) continue;
        f_map[i] = new_nf;
        if (new_nf != i) hm.faces[new_nf] = hm.faces[i];
        ++new_nf;
    }
    hm.faces.resize(new_nf);

    // Remap halfedges — keep only those belonging to live faces
    int nh = (int)hm.hedges.size();
    std::vector<int> h_map(nh, -1);
    int new_nh = 0;
    for (int i = 0; i < nh; ++i) {
        int f = hm.hedges[i].face;
        if (f < 0 || f_map[f] < 0) continue;
        h_map[i] = new_nh;
        if (new_nh != i) hm.hedges[new_nh] = hm.hedges[i];
        ++new_nh;
    }
    hm.hedges.resize(new_nh);

    // Apply remaps
    for (auto& h : hm.hedges) {
        h.next = h_map[h.next];
        h.twin = (h.twin >= 0) ? h_map[h.twin] : -1;
        h.vertex = v_map[h.vertex];
        h.face = f_map[h.face];
    }
    for (auto& f : hm.faces) {
        f.halfedge = h_map[f.halfedge];
    }
    for (auto& v : hm.verts) {
        v.halfedge = (v.halfedge >= 0) ? h_map[v.halfedge] : -1;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
// to_output_mesh
///////////////////////////////////////////////////////////////////////////////////////////

Mesh TrimeshRemesh::to_output_mesh(const HalfedgeMesh& hm) const {
    Mesh result;
    for (int vi = 0; vi < (int)hm.verts.size(); ++vi) {
        const auto& v = hm.verts[vi];
        if (v.flags & RVertex::DELETED) continue;
        size_t vk = result.add_vertex(Point(v.px, v.py, v.pz));
        result.vertex[vk].set_normal(v.nx, v.ny, v.nz);
    }
    for (int fi = 0; fi < (int)hm.faces.size(); ++fi) {
        if (hm.faces[fi].halfedge < 0) continue;
        int h0 = hm.faces[fi].halfedge;
        int h1 = hm.hedges[h0].next;
        int h2 = hm.hedges[h1].next;
        int va = hm.hedges[h2].vertex;
        int vb = hm.hedges[h0].vertex;
        int vc = hm.hedges[h1].vertex;
        result.add_face({(size_t)va, (size_t)vb, (size_t)vc});
    }
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// mesh() — main loop
///////////////////////////////////////////////////////////////////////////////////////////

Mesh TrimeshRemesh::mesh() const {
    double bbox_diag = compute_bbox_diagonal();
    double target = m_max_edge_length > 0 ? m_max_edge_length : bbox_diag / 30.0;
    double max_len = target * (4.0 / 3.0);
    double min_len = target * (4.0 / 5.0);

    HalfedgeMesh hm;
    std::cerr << "  build_initial_mesh..." << std::endl;
    build_initial_mesh(hm);
    std::cerr << "  built: " << hm.verts.size() << " verts, " << hm.faces.size() << " faces, " << hm.hedges.size() << " hedges" << std::endl;

    for (int iter = 0; iter < m_iterations; ++iter) {
        std::cerr << "  iter " << iter << " split..." << std::endl;
        split_long_edges(hm, max_len);
        std::cerr << "  collapse..." << std::endl;
        collapse_short_edges(hm, min_len);
        std::cerr << "  flip..." << std::endl;
        flip_edges(hm);
        std::cerr << "  compact..." << std::endl;
        compact(hm);
        if (iter > 8) {
            std::cerr << "  smooth..." << std::endl;
            smooth_vertices(hm, 0.8);
        }
        std::cerr << "  project..." << std::endl;
        project_to_surface(hm);
        std::cerr << "  done: " << hm.verts.size() << "v " << hm.faces.size() << "f" << std::endl;
    }
    return to_output_mesh(hm);
}

} // namespace session_cpp
