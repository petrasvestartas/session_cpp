#include "trimesh_grid.h"
#include <cmath>
#include <algorithm>

namespace session_cpp {

TrimeshGrid::TrimeshGrid(const NurbsSurface& surface)
    : m_surface(surface) {}

TrimeshGrid& TrimeshGrid::set_max_angle(double degrees) {
    m_max_angle = degrees;
    return *this;
}

TrimeshGrid& TrimeshGrid::set_max_edge_length(double length) {
    m_max_edge_length = length;
    return *this;
}

TrimeshGrid& TrimeshGrid::set_min_edge_length(double length) {
    m_min_edge_length = length;
    return *this;
}

TrimeshGrid& TrimeshGrid::set_max_chord_height(double height) {
    m_max_chord_height = height;
    return *this;
}

double TrimeshGrid::compute_bbox_diagonal() const {
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

Mesh TrimeshGrid::mesh() const {
    std::vector<double> usp = m_surface.get_span_vector(0);
    std::vector<double> vsp = m_surface.get_span_vector(1);
    int ns_u = (int)usp.size() - 1, ns_v = (int)vsp.size() - 1;

    double max_angle_deg = m_max_angle;
    if (m_max_chord_height > 0) {
        double diag = compute_bbox_diagonal();
        double ratio = m_max_chord_height / diag;
        double chord_angle = std::max(3.0, std::min(ratio * 800.0, 30.0));
        max_angle_deg = std::min(max_angle_deg, chord_angle);
    }

    int deg_u = m_surface.degree(0), deg_v = m_surface.degree(1);
    double bbox_diag = compute_bbox_diagonal();
    double flat_tol = bbox_diag * 0.001;

    auto span_flatness = [&](int span_u, int span_v) -> double {
        Point c00 = m_surface.get_cv(span_u, span_v);
        Point c10 = m_surface.get_cv(span_u + deg_u, span_v);
        Point c01 = m_surface.get_cv(span_u, span_v + deg_v);
        Point c11 = m_surface.get_cv(span_u + deg_u, span_v + deg_v);
        double max_dist = 0.0;
        for (int a = 0; a <= deg_u; ++a) {
            double s = (deg_u > 0) ? (double)a / deg_u : 0.0;
            for (int b = 0; b <= deg_v; ++b) {
                if ((a == 0 || a == deg_u) && (b == 0 || b == deg_v)) continue;
                double t = (deg_v > 0) ? (double)b / deg_v : 0.0;
                Point cv = m_surface.get_cv(span_u + a, span_v + b);
                double bx = (1-s)*(1-t)*c00[0] + s*(1-t)*c10[0] + (1-s)*t*c01[0] + s*t*c11[0];
                double by = (1-s)*(1-t)*c00[1] + s*(1-t)*c10[1] + (1-s)*t*c01[1] + s*t*c11[1];
                double bz = (1-s)*(1-t)*c00[2] + s*(1-t)*c10[2] + (1-s)*t*c01[2] + s*t*c11[2];
                double dx = cv[0]-bx, dy = cv[1]-by, dz = cv[2]-bz;
                double d = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (d > max_dist) max_dist = d;
            }
        }
        return max_dist;
    };

    std::vector<double> u_flat(ns_u, 0.0), v_flat(ns_v, 0.0);
    for (int i = 0; i < ns_u; ++i)
        for (int j = 0; j < ns_v; ++j) {
            double f = span_flatness(i, j);
            if (f > u_flat[i]) u_flat[i] = f;
            if (f > v_flat[j]) v_flat[j] = f;
        }

    auto span_subs = [&](int dir, const std::vector<double>& sp,
                         const std::vector<double>& osp) -> std::vector<int> {
        int n = (int)sp.size() - 1;
        std::vector<int> subs(n, 1);
        double s_positions[3] = {
            osp.front() + (osp.back() - osp.front()) * 0.25,
            (osp.front() + osp.back()) * 0.5,
            osp.front() + (osp.back() - osp.front()) * 0.75
        };
        for (int i = 0; i < n; ++i) {
            double t0 = sp[i], t1 = sp[i + 1];
            double max_angle = 0.0;
            for (int si = 0; si < 3; ++si) {
                double s = s_positions[si];
                double prev_nx, prev_ny, prev_nz;
                double total_angle = 0.0;
                for (int k = 0; k <= 4; ++k) {
                    double t = t0 + k * (t1 - t0) / 4.0;
                    double px, py, pz, nx, ny, nz;
                    if (dir == 0) m_surface.point_and_normal_at(t, s, px,py,pz, nx,ny,nz);
                    else          m_surface.point_and_normal_at(s, t, px,py,pz, nx,ny,nz);
                    if (k > 0) {
                        double dot = prev_nx*nx + prev_ny*ny + prev_nz*nz;
                        dot = std::max(-1.0, std::min(1.0, dot));
                        total_angle += std::acos(dot) * 180.0 / Tolerance::PI;
                    }
                    prev_nx = nx; prev_ny = ny; prev_nz = nz;
                }
                if (total_angle > max_angle) max_angle = total_angle;
            }
            subs[i] = std::max(1, std::min((int)std::ceil(max_angle / max_angle_deg), 24));

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
        }
        return subs;
    };

    std::vector<int> u_subs = span_subs(0, usp, vsp);
    std::vector<int> v_subs = span_subs(1, vsp, usp);

    // Arc-length aspect ratio balancing
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
            if (ratio > 2.0) {
                double scale = std::sqrt(ratio);
                for (int& s : u_subs) s = std::min((int)std::ceil(s * scale), 24);
            } else if (ratio < 0.5) {
                double scale = std::sqrt(1.0 / ratio);
                for (int& s : v_subs) s = std::min((int)std::ceil(s * scale), 24);
            }
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
    if (closed_u) us.pop_back();
    if (closed_v) vs.pop_back();
    int nu = (int)us.size(), nv = (int)vs.size();
    Mesh result;
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) {
            double px, py, pz;
            m_surface.point_at(us[i], vs[j], px, py, pz);
            result.add_vertex(Point(px, py, pz));
        }
    int nu_faces = closed_u ? nu : nu - 1;
    int nv_faces = closed_v ? nv : nv - 1;
    for (int i = 0; i < nu_faces; ++i)
        for (int j = 0; j < nv_faces; ++j) {
            int i1 = (i + 1) % nu, j1 = (j + 1) % nv;
            size_t v00 = i * nv + j, v10 = i1 * nv + j;
            size_t v01 = i * nv + j1, v11 = i1 * nv + j1;
            if ((i + j) % 2 == 0) {
                result.add_face({v00, v10, v11});
                result.add_face({v00, v11, v01});
            } else {
                result.add_face({v00, v10, v01});
                result.add_face({v10, v11, v01});
            }
        }
    // Compute vertex normals from face normals
    int nv_total = (int)result.vertex.size();
    std::vector<double> vnx(nv_total, 0.0), vny(nv_total, 0.0), vnz(nv_total, 0.0);
    for (auto& [fi, vids] : result.face) {
        if (vids.size() < 3) continue;
        auto& p0 = result.vertex.at(vids[0]);
        auto& p1 = result.vertex.at(vids[1]);
        auto& p2 = result.vertex.at(vids[2]);
        double e1x = p1.x-p0.x, e1y = p1.y-p0.y, e1z = p1.z-p0.z;
        double e2x = p2.x-p0.x, e2y = p2.y-p0.y, e2z = p2.z-p0.z;
        double fnx = e1y*e2z - e1z*e2y, fny = e1z*e2x - e1x*e2z, fnz = e1x*e2y - e1y*e2x;
        for (auto vi : vids) { vnx[vi] += fnx; vny[vi] += fny; vnz[vi] += fnz; }
    }
    for (int i = 0; i < nv_total; ++i) {
        double len = std::sqrt(vnx[i]*vnx[i] + vny[i]*vny[i] + vnz[i]*vnz[i]);
        if (len > 1e-15) { vnx[i] /= len; vny[i] /= len; vnz[i] /= len; }
        result.vertex[i].set_normal(vnx[i], vny[i], vnz[i]);
    }
    return result;
}

} // namespace session_cpp
