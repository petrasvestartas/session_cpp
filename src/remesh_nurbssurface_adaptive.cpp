#include "remesh_nurbssurface_adaptive.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <map>

namespace session_cpp {

RemeshNurbssurfaceAdaptive::RemeshNurbssurfaceAdaptive(const NurbsSurface& surface)
    : m_surface(surface) {}

RemeshNurbssurfaceAdaptive& RemeshNurbssurfaceAdaptive::set_max_angle(double degrees) {
    m_max_angle = degrees;
    return *this;
}

RemeshNurbssurfaceAdaptive& RemeshNurbssurfaceAdaptive::set_max_edge_length(double length) {
    m_max_edge_length = length;
    return *this;
}

RemeshNurbssurfaceAdaptive& RemeshNurbssurfaceAdaptive::set_min_edge_length(double length) {
    m_min_edge_length = length;
    return *this;
}

RemeshNurbssurfaceAdaptive& RemeshNurbssurfaceAdaptive::set_max_chord_height(double height) {
    m_max_chord_height = height;
    return *this;
}

double RemeshNurbssurfaceAdaptive::compute_bbox_diagonal() const {
    double minx = 1e30, miny = 1e30, minz = 1e30;
    double maxx = -1e30, maxy = -1e30, maxz = -1e30;
    for (int i = 0; i < m_surface.cv_count(0); ++i)
        for (int j = 0; j < m_surface.cv_count(1); ++j) {
            Point p = m_surface.get_cv(i, j);
            if (p[0] < minx) minx = p[0]; if (p[1] < miny) miny = p[1]; if (p[2] < minz) minz = p[2];
            if (p[0] > maxx) maxx = p[0]; if (p[1] > maxy) maxy = p[1]; if (p[2] > maxz) maxz = p[2];
        }
    double dx = maxx - minx, dy = maxy - miny, dz = maxz - minz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Mesh RemeshNurbssurfaceAdaptive::mesh() const {
    auto usp = m_surface.get_span_vector(0);
    auto vsp = m_surface.get_span_vector(1);
    int ns_u = (int)usp.size() - 1, ns_v = (int)vsp.size() - 1;
    double bbox_diag = compute_bbox_diagonal();

    // Convert max_angle to squared normal difference threshold
    // |n1-n2|^2 = 2 - 2*cos(theta)
    double norm_tol = 2.0 - 2.0 * std::cos(m_max_angle * Tolerance::PI / 180.0);
    double chord_tol = (m_max_chord_height > 0) ? m_max_chord_height : bbox_diag * 0.005;
    bool closed_u = m_surface.is_closed(0), closed_v = m_surface.is_closed(1);
    bool sing_v0 = m_surface.is_singular(0), sing_v1 = m_surface.is_singular(2);
    int max_depth = 8;

    // === Node structure ===
    struct Corner { double px, py, pz, nx, ny, nz; };
    struct Node {
        double u0, v0, u1, v1;
        Corner c[5]; // SW=0, SE=1, NE=2, NW=3, Center=4
        int ch[4];   // children indices, -1 = none
        int depth;
        bool leaf() const { return ch[0] < 0 && ch[2] < 0; }
    };

    std::vector<Node> nodes;
    nodes.reserve(ns_u * ns_v * 256);

    auto eval = [&](double u, double v) -> Corner {
        Corner c = {};
        m_surface.point_at(u, v, c.px, c.py, c.pz);
        // Compute normal from derivatives directly (avoids (0,0,1) fallback for degenerate poles)
        auto derivs = m_surface.evaluate(u, v, 1);
        if (derivs.size() >= 3) {
            double dux = derivs[1][0], duy = derivs[1][1], duz = derivs[1][2];
            double dvx = derivs[2][0], dvy = derivs[2][1], dvz = derivs[2][2];
            double nx = dvy * duz - dvz * duy;
            double ny = dvz * dux - dvx * duz;
            double nz = dvx * duy - dvy * dux;
            double len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 1e-10) { c.nx = nx / len; c.ny = ny / len; c.nz = nz / len; }
        }
        return c;
    };

    auto ndiff = [](const Corner& a, const Corner& b) -> double {
        double dx = a.nx - b.nx, dy = a.ny - b.ny, dz = a.nz - b.nz;
        return dx * dx + dy * dy + dz * dz;
    };
    auto nvalid = [](const Corner& c) -> bool {
        return c.nx * c.nx + c.ny * c.ny + c.nz * c.nz > 1e-20;
    };
    auto pdist2 = [](const Corner& a, const Corner& b) -> double {
        double dx = a.px - b.px, dy = a.py - b.py, dz = a.pz - b.pz;
        return dx * dx + dy * dy + dz * dz;
    };
    auto pmid = [](const Corner& a, const Corner& b) -> Corner {
        return {(a.px + b.px) * 0.5, (a.py + b.py) * 0.5, (a.pz + b.pz) * 0.5, 0, 0, 0};
    };

    // Fill child node at index idx
    auto fill_child = [&](int idx, double u0, double v0, double u1, double v1,
                          Corner sw, Corner se, Corner ne, Corner nw, int depth) {
        nodes[idx].u0 = u0; nodes[idx].v0 = v0; nodes[idx].u1 = u1; nodes[idx].v1 = v1;
        nodes[idx].c[0] = sw; nodes[idx].c[1] = se; nodes[idx].c[2] = ne; nodes[idx].c[3] = nw;
        nodes[idx].c[4] = eval((u0 + u1) * 0.5, (v0 + v1) * 0.5);
        nodes[idx].ch[0] = nodes[idx].ch[1] = nodes[idx].ch[2] = nodes[idx].ch[3] = -1;
        nodes[idx].depth = depth;
    };

    // Recursive subdivision: compute edge midpoints, decide direction, then split
    auto subdivide = [&](auto& self, int idx) -> void {
        if (nodes[idx].depth >= max_depth) return;
        Node p = nodes[idx]; // copy (vector may realloc)
        double um = (p.u0 + p.u1) * 0.5, vm = (p.v0 + p.v1) * 0.5;

        // --- Min edge length guard ---
        if (m_min_edge_length > 0) {
            double me = 0;
            for (int i = 0; i < 4; i++) me = std::max(me, pdist2(p.c[i], p.c[(i + 1) % 4]));
            if (me < m_min_edge_length * m_min_edge_length) return;
        }

        // --- Normal deviation on edges → directional ---
        // Bottom(SW-SE) & Top(NE-NW) vary in U → split_u
        // Right(SE-NE) & Left(NW-SW) vary in V → split_v
        bool split_u = false, split_v = false;
        if (nvalid(p.c[0]) && nvalid(p.c[1]) && ndiff(p.c[0], p.c[1]) > norm_tol) split_u = true;
        if (nvalid(p.c[2]) && nvalid(p.c[3]) && ndiff(p.c[2], p.c[3]) > norm_tol) split_u = true;
        if (nvalid(p.c[1]) && nvalid(p.c[2]) && ndiff(p.c[1], p.c[2]) > norm_tol) split_v = true;
        if (nvalid(p.c[3]) && nvalid(p.c[0]) && ndiff(p.c[3], p.c[0]) > norm_tol) split_v = true;

        // --- Directional chord height using edge midpoints ---
        // Evaluate only the midpoints needed for undecided directions
        Corner s_mid = {}, n_mid = {}, w_mid = {}, e_mid = {};
        bool have_sn = false, have_we = false;
        if (!split_u) {
            s_mid = eval(um, p.v0); n_mid = eval(um, p.v1); have_sn = true;
            // U-chord: edge mid vs linear interp of its endpoints
            Corner s_lin = pmid(p.c[0], p.c[1]); // avg(SW,SE) at bottom edge
            Corner n_lin = pmid(p.c[3], p.c[2]); // avg(NW,NE) at top edge
            if (pdist2(s_mid, s_lin) > chord_tol * chord_tol) split_u = true;
            if (pdist2(n_mid, n_lin) > chord_tol * chord_tol) split_u = true;
            // Also check: normal at edge midpoints vs adjacent corners
            if (nvalid(s_mid) && nvalid(p.c[0]) && ndiff(s_mid, p.c[0]) > norm_tol) split_u = true;
            if (nvalid(n_mid) && nvalid(p.c[3]) && ndiff(n_mid, p.c[3]) > norm_tol) split_u = true;
        }
        if (!split_v) {
            w_mid = eval(p.u0, vm); e_mid = eval(p.u1, vm); have_we = true;
            Corner w_lin = pmid(p.c[0], p.c[3]); // avg(SW,NW) at left edge
            Corner e_lin = pmid(p.c[1], p.c[2]); // avg(SE,NE) at right edge
            if (pdist2(w_mid, w_lin) > chord_tol * chord_tol) split_v = true;
            if (pdist2(e_mid, e_lin) > chord_tol * chord_tol) split_v = true;
            if (nvalid(w_mid) && nvalid(p.c[0]) && ndiff(w_mid, p.c[0]) > norm_tol) split_v = true;
            if (nvalid(e_mid) && nvalid(p.c[1]) && ndiff(e_mid, p.c[1]) > norm_tol) split_v = true;
        }

        // --- Bilinear twist: center vs diagonal midpoints (catches saddle surfaces) ---
        // Only when NEITHER edge check triggered; looser tolerance (diagonal is ~1.4x edge)
        if (!split_u && !split_v) {
            double twist_tol2 = 4.0 * chord_tol * chord_tol;
            for (int d = 0; d < 2; d++) {
                Corner mid = pmid(p.c[d], p.c[d + 2]);
                if (pdist2(p.c[4], mid) > twist_tol2) { split_u = true; split_v = true; }
            }
        }

        // --- Max edge length per direction ---
        if (m_max_edge_length > 0) {
            double me = m_max_edge_length * m_max_edge_length;
            if (pdist2(p.c[0], p.c[1]) > me || pdist2(p.c[2], p.c[3]) > me) split_u = true;
            if (pdist2(p.c[1], p.c[2]) > me || pdist2(p.c[3], p.c[0]) > me) split_v = true;
        }

        // --- Analytical curvature at cell center ---
        if (!split_u || !split_v) {
            auto derivs2 = m_surface.evaluate(um, vm, 2);
            if (derivs2.size() >= 6) {
                double dux = derivs2[2][0], duy = derivs2[2][1], duz = derivs2[2][2];
                double dvx = derivs2[1][0], dvy = derivs2[1][1], dvz = derivs2[1][2];
                double d2ux = derivs2[5][0], d2uy = derivs2[5][1], d2uz = derivs2[5][2];
                double d2vx = derivs2[3][0], d2vy = derivs2[3][1], d2vz = derivs2[3][2];
                double nx = duy * dvz - duz * dvy;
                double ny = duz * dvx - dux * dvz;
                double nz = dux * dvy - duy * dvx;
                double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
                double E = dux*dux + duy*duy + duz*duz;
                double G = dvx*dvx + dvy*dvy + dvz*dvz;
                if (nlen > 1e-10 && E > 1e-20 && G > 1e-20) {
                    double inv_nlen = 1.0 / nlen;
                    double nnx = nx * inv_nlen, nny = ny * inv_nlen, nnz = nz * inv_nlen;
                    double e_coeff = d2ux*nnx + d2uy*nny + d2uz*nnz;
                    double g_coeff = d2vx*nnx + d2vy*nny + d2vz*nnz;
                    double kappa_u = std::abs(e_coeff) / E;
                    double kappa_v = std::abs(g_coeff) / G;
                    double span_u = std::sqrt(E) * (p.u1 - p.u0);
                    double span_v = std::sqrt(G) * (p.v1 - p.v0);
                    if (!split_u && kappa_u > 1e-20) {
                        double needed_u = span_u * std::sqrt(kappa_u / (8.0 * chord_tol));
                        if (needed_u > 1.0) split_u = true;
                    }
                    if (!split_v && kappa_v > 1e-20) {
                        double needed_v = span_v * std::sqrt(kappa_v / (8.0 * chord_tol));
                        if (needed_v > 1.0) split_v = true;
                    }
                }
            }
        }

        if (!split_u && !split_v) return;

        int d = p.depth + 1;
        if (split_u && split_v) { // BOTH: 4 children
            if (!have_sn) { s_mid = eval(um, p.v0); n_mid = eval(um, p.v1); }
            if (!have_we) { w_mid = eval(p.u0, vm); e_mid = eval(p.u1, vm); }
            int b = (int)nodes.size();
            nodes.resize(b + 4);
            nodes[idx].ch[0] = b; nodes[idx].ch[1] = b + 1; nodes[idx].ch[2] = b + 2; nodes[idx].ch[3] = b + 3;
            fill_child(b,     p.u0, p.v0, um,   vm,   p.c[0], s_mid,  p.c[4], w_mid,  d);
            fill_child(b + 1, um,   p.v0, p.u1, vm,   s_mid,  p.c[1], e_mid,  p.c[4], d);
            fill_child(b + 2, um,   vm,   p.u1, p.v1, p.c[4], e_mid,  p.c[2], n_mid,  d);
            fill_child(b + 3, p.u0, vm,   um,   p.v1, w_mid,  p.c[4], n_mid,  p.c[3], d);
            for (int ci = 0; ci < 4; ci++) self(self, b + ci);
        } else if (split_u) { // U only: 2 children (left, right)
            if (!have_sn) { s_mid = eval(um, p.v0); n_mid = eval(um, p.v1); }
            int b = (int)nodes.size();
            nodes.resize(b + 2);
            nodes[idx].ch[0] = b; nodes[idx].ch[1] = b + 1; nodes[idx].ch[2] = -1; nodes[idx].ch[3] = -1;
            fill_child(b,     p.u0, p.v0, um,   p.v1, p.c[0], s_mid,  n_mid,  p.c[3], d);
            fill_child(b + 1, um,   p.v0, p.u1, p.v1, s_mid,  p.c[1], p.c[2], n_mid,  d);
            for (int ci = 0; ci < 2; ci++) self(self, b + ci);
        } else { // V only: 2 children (bottom, top)
            if (!have_we) { w_mid = eval(p.u0, vm); e_mid = eval(p.u1, vm); }
            int b = (int)nodes.size();
            nodes.resize(b + 2);
            nodes[idx].ch[0] = b; nodes[idx].ch[1] = -1; nodes[idx].ch[2] = b + 1; nodes[idx].ch[3] = -1;
            fill_child(b,     p.u0, p.v0, p.u1, vm,   p.c[0], p.c[1], e_mid,  w_mid,  d);
            fill_child(b + 1, p.u0, vm,   p.u1, p.v1, w_mid,  e_mid,  p.c[2], p.c[3], d);
            for (int ci : {b, b + 1}) self(self, ci);
        }
    };

    // === Create initial nodes per span pair ===
    // Pre-evaluate grid intersections to avoid redundant evaluations
    int ng_u = ns_u + 1, ng_v = ns_v + 1;
    std::vector<Corner> grid(ng_u * ng_v);
    for (int i = 0; i < ng_u; i++)
        for (int j = 0; j < ng_v; j++)
            grid[i * ng_v + j] = eval(usp[i], vsp[j]);

    for (int i = 0; i < ns_u; i++)
        for (int j = 0; j < ns_v; j++) {
            int idx = (int)nodes.size();
            Corner ct = eval((usp[i] + usp[i + 1]) * 0.5, (vsp[j] + vsp[j + 1]) * 0.5);
            Node nd;
            nd.u0 = usp[i]; nd.v0 = vsp[j]; nd.u1 = usp[i + 1]; nd.v1 = vsp[j + 1];
            nd.c[0] = grid[i * ng_v + j];           // SW
            nd.c[1] = grid[(i + 1) * ng_v + j];     // SE
            nd.c[2] = grid[(i + 1) * ng_v + (j + 1)]; // NE
            nd.c[3] = grid[i * ng_v + (j + 1)];     // NW
            nd.c[4] = ct;
            nd.ch[0] = nd.ch[1] = nd.ch[2] = nd.ch[3] = -1;
            nd.depth = 0;
            nodes.push_back(nd);
            subdivide(subdivide, idx);
        }

    // === Collect leaf corner UVs for T-junction detection ===
    auto qk = [](double x) -> int64_t { return (int64_t)std::round(x * 1e10); };
    auto norm_u = [&](double u) -> double {
        return (closed_u && std::abs(u - usp.back()) < 1e-10) ? usp.front() : u;
    };
    auto norm_v = [&](double v) -> double {
        return (closed_v && std::abs(v - vsp.back()) < 1e-10) ? vsp.front() : v;
    };

    using IKey = std::pair<int64_t, int64_t>;
    std::map<IKey, Corner> key_corner; // UV key -> Corner data
    std::map<int64_t, std::vector<int64_t>> by_v, by_u; // for edge queries

    for (auto& nd : nodes) {
        if (!nd.leaf()) continue;
        double us[4] = {nd.u0, nd.u1, nd.u1, nd.u0};
        double vs[4] = {nd.v0, nd.v0, nd.v1, nd.v1};
        for (int ci = 0; ci < 4; ci++) {
            double un = norm_u(us[ci]), vn = norm_v(vs[ci]);
            IKey key = {qk(un), qk(vn)};
            if (key_corner.find(key) == key_corner.end())
                key_corner[key] = nd.c[ci];
            by_v[key.second].push_back(qk(us[ci])); // raw u for range queries
            by_u[key.first].push_back(qk(vs[ci]));  // raw v for range queries
        }
    }
    for (auto& [k, vec] : by_v) {
        std::sort(vec.begin(), vec.end());
        vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
    }
    for (auto& [k, vec] : by_u) {
        std::sort(vec.begin(), vec.end());
        vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
    }

    // === Build mesh ===
    Mesh result;
    size_t south_pole = SIZE_MAX, north_pole = SIZE_MAX;
    if (sing_v0) {
        Corner c0 = eval(usp.front(), vsp.front());
        south_pole = result.add_vertex(Point(c0.px, c0.py, c0.pz));
    }
    if (sing_v1) {
        Corner c1 = eval(usp.front(), vsp.back());
        north_pole = result.add_vertex(Point(c1.px, c1.py, c1.pz));
    }

    std::map<IKey, size_t> vtx_map;
    auto get_vtx = [&](double u, double v) -> size_t {
        if (sing_v0 && std::abs(v - vsp.front()) < 1e-10) return south_pole;
        if (sing_v1 && std::abs(v - vsp.back()) < 1e-10) return north_pole;
        u = norm_u(u); v = norm_v(v);
        IKey key = {qk(u), qk(v)};
        auto it = vtx_map.find(key);
        if (it != vtx_map.end()) return it->second;
        auto cit = key_corner.find(key);
        Corner c;
        if (cit != key_corner.end()) c = cit->second;
        else { c = eval(u, v); key_corner[key] = c; }
        size_t vi = result.add_vertex(Point(c.px, c.py, c.pz));
        result.vertex[vi].attributes["u"] = u;
        result.vertex[vi].attributes["v"] = v;
        vtx_map[key] = vi;
        return vi;
    };

    // Find T-junction midpoints on a horizontal edge (same v, from u_start to u_end)
    auto find_h_mids = [&](double u_start, double u_end, double v_const) -> std::vector<double> {
        std::vector<double> res;
        int64_t qv = qk(norm_v(v_const));
        auto it = by_v.find(qv);
        if (it == by_v.end()) return res;
        int64_t qa = qk(u_start), qb = qk(u_end); // raw u for range query
        int64_t lo = std::min(qa, qb), hi = std::max(qa, qb);
        auto lit = std::upper_bound(it->second.begin(), it->second.end(), lo);
        auto hit = std::lower_bound(it->second.begin(), it->second.end(), hi);
        for (auto uit = lit; uit != hit; ++uit)
            res.push_back(*uit / 1e10);
        if (qa > qb) std::reverse(res.begin(), res.end());
        return res;
    };

    // Find T-junction midpoints on a vertical edge (same u, from v_start to v_end)
    auto find_v_mids = [&](double u_const, double v_start, double v_end) -> std::vector<double> {
        std::vector<double> res;
        int64_t qu = qk(norm_u(u_const));
        auto it = by_u.find(qu);
        if (it == by_u.end()) return res;
        int64_t qa = qk(v_start), qb = qk(v_end); // raw v for range query
        int64_t lo = std::min(qa, qb), hi = std::max(qa, qb);
        auto lit = std::upper_bound(it->second.begin(), it->second.end(), lo);
        auto hit = std::lower_bound(it->second.begin(), it->second.end(), hi);
        for (auto vit = lit; vit != hit; ++vit)
            res.push_back(*vit / 1e10);
        if (qa > qb) std::reverse(res.begin(), res.end());
        return res;
    };

    // === Triangulate leaves ===
    for (auto& nd : nodes) {
        if (!nd.leaf()) continue;

        // Build ordered polygon walking CCW around leaf boundary
        std::vector<size_t> poly;

        // SW corner
        poly.push_back(get_vtx(nd.u0, nd.v0));
        // Bottom edge: SW→SE (increasing u at v=v0)
        for (double u : find_h_mids(nd.u0, nd.u1, nd.v0))
            poly.push_back(get_vtx(u, nd.v0));
        // SE corner
        poly.push_back(get_vtx(nd.u1, nd.v0));
        // Right edge: SE→NE (increasing v at u=u1)
        for (double v : find_v_mids(nd.u1, nd.v0, nd.v1))
            poly.push_back(get_vtx(nd.u1, v));
        // NE corner
        poly.push_back(get_vtx(nd.u1, nd.v1));
        // Top edge: NE→NW (decreasing u at v=v1)
        for (double u : find_h_mids(nd.u1, nd.u0, nd.v1))
            poly.push_back(get_vtx(u, nd.v1));
        // NW corner
        poly.push_back(get_vtx(nd.u0, nd.v1));
        // Left edge: NW→SW (decreasing v at u=u0)
        for (double v : find_v_mids(nd.u0, nd.v1, nd.v0))
            poly.push_back(get_vtx(nd.u0, v));

        // Remove consecutive duplicate vertices (poles, seams)
        poly.erase(std::unique(poly.begin(), poly.end()), poly.end());
        while (poly.size() > 1 && poly.front() == poly.back()) poly.pop_back();

        int n = (int)poly.size();
        if (n < 3) continue;

        if (n == 3) {
            result.add_face({poly[0], poly[1], poly[2]});
        } else if (n == 4) {
            // Shorter diagonal produces more equilateral triangles
            Point pos0 = result.vertex.at(poly[0]).position(), pos1 = result.vertex.at(poly[1]).position();
            Point pos2 = result.vertex.at(poly[2]).position(), pos3 = result.vertex.at(poly[3]).position();
            double d02 = (pos0[0]-pos2[0])*(pos0[0]-pos2[0]) + (pos0[1]-pos2[1])*(pos0[1]-pos2[1]) + (pos0[2]-pos2[2])*(pos0[2]-pos2[2]);
            double d13 = (pos1[0]-pos3[0])*(pos1[0]-pos3[0]) + (pos1[1]-pos3[1])*(pos1[1]-pos3[1]) + (pos1[2]-pos3[2])*(pos1[2]-pos3[2]);
            if (d02 <= d13) {
                result.add_face({poly[0], poly[1], poly[2]});
                result.add_face({poly[0], poly[2], poly[3]});
            } else {
                result.add_face({poly[0], poly[1], poly[3]});
                result.add_face({poly[1], poly[2], poly[3]});
            }
        } else {
            // Fan triangulation from center for polygons with T-junctions
            double cu = norm_u((nd.u0 + nd.u1) * 0.5);
            double cv = norm_v((nd.v0 + nd.v1) * 0.5);
            IKey ck = {qk(cu), qk(cv)};
            if (key_corner.find(ck) == key_corner.end())
                key_corner[ck] = nd.c[4];
            size_t cvi = get_vtx(cu, cv);
            for (int i = 0; i < n; i++) {
                int j = (i + 1) % n;
                if (poly[i] != poly[j] && poly[i] != cvi && poly[j] != cvi)
                    result.add_face({poly[i], poly[j], cvi});
            }
        }
    }

    // === Compute vertex normals from face normals ===
    int nv = (int)result.vertex.size();
    std::vector<double> vnx(nv, 0.0), vny(nv, 0.0), vnz(nv, 0.0);
    for (auto& [fi, vids] : result.face) {
        if (vids.size() < 3) continue;
        Point pos0 = result.vertex.at(vids[0]).position();
        Point pos1 = result.vertex.at(vids[1]).position();
        Point pos2 = result.vertex.at(vids[2]).position();
        double e1x = pos1[0] - pos0[0], e1y = pos1[1] - pos0[1], e1z = pos1[2] - pos0[2];
        double e2x = pos2[0] - pos0[0], e2y = pos2[1] - pos0[1], e2z = pos2[2] - pos0[2];
        double fnx = e1y * e2z - e1z * e2y;
        double fny = e1z * e2x - e1x * e2z;
        double fnz = e1x * e2y - e1y * e2x;
        for (auto vi : vids) { vnx[vi] += fnx; vny[vi] += fny; vnz[vi] += fnz; }
    }
    for (int i = 0; i < nv; i++) {
        double len = std::sqrt(vnx[i] * vnx[i] + vny[i] * vny[i] + vnz[i] * vnz[i]);
        if (len > 1e-15) { vnx[i] /= len; vny[i] /= len; vnz[i] /= len; }
        result.vertex[i].set_normal(vnx[i], vny[i], vnz[i]);
    }
    return result;
}

} // namespace session_cpp
