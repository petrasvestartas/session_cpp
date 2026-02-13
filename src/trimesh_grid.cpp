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
                double max_angle = 0.0;
                for (int si = 0; si < n_other; ++si) {
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
            }

            // Direct chord-height deviation check
            {
                double chord_tol = (m_max_chord_height > 0) ? m_max_chord_height : bbox_diag * 0.005;
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
                if (max_dev > chord_tol) {
                    int chord_subs = std::max(2, (int)std::ceil(std::sqrt(max_dev / chord_tol)));
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
            if (ratio > 2.0 && deg_u > 1) {
                double scale = std::sqrt(ratio);
                for (int& s : u_subs) s = std::min((int)std::ceil(s * scale), 24);
            } else if (ratio < 0.5 && deg_v > 1) {
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

    // For closed surfaces, ensure the wrapping gap is not disproportionately large.
    // The last span's subdivision might be lower than others, causing the closing
    // seam faces to span a gap much larger than interior faces.
    auto fix_closed_gap = [](std::vector<double>& params, const std::vector<double>& spans, bool closed) {
        if (!closed || params.size() < 3) return;
        params.pop_back(); // Remove duplicate endpoint (maps to same point as first)
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

    // Detect singular edges (collapsed to a single point)
    bool sing_v0 = m_surface.is_singular(0); // south: v=vs[0]
    bool sing_v1 = m_surface.is_singular(2); // north: v=vs[nv-1]
    int j_start = sing_v0 ? 1 : 0;
    int j_end = sing_v1 ? nv - 1 : nv;
    int nv_grid = j_end - j_start;

    Mesh result;
    size_t south_pole = 0, north_pole = 0;
    if (sing_v0) {
        double px, py, pz;
        m_surface.point_at(us[0], vs[0], px, py, pz);
        south_pole = result.add_vertex(Point(px, py, pz));
    }
    if (sing_v1) {
        double px, py, pz;
        m_surface.point_at(us[0], vs[nv - 1], px, py, pz);
        north_pole = result.add_vertex(Point(px, py, pz));
    }
    size_t grid_base = result.vertex.size();
    for (int i = 0; i < nu; ++i)
        for (int j = j_start; j < j_end; ++j) {
            double px, py, pz;
            m_surface.point_at(us[i], vs[j], px, py, pz);
            result.add_vertex(Point(px, py, pz));
        }

    auto grid_idx = [&](int i, int j) -> size_t {
        return grid_base + (size_t)i * nv_grid + (j - j_start);
    };

    int nu_faces = closed_u ? nu : nu - 1;

    // South pole fan
    if (sing_v0) {
        for (int i = 0; i < nu_faces; ++i) {
            int i1 = (i + 1) % nu;
            result.add_face({south_pole, grid_idx(i1, j_start), grid_idx(i, j_start)});
        }
    }

    // Interior grid faces
    int nv_interior = nv_grid - 1;
    if (closed_v && !sing_v0 && !sing_v1) nv_interior = nv_grid;
    for (int i = 0; i < nu_faces; ++i)
        for (int jj = 0; jj < nv_interior; ++jj) {
            int j = jj + j_start;
            int i1 = (i + 1) % nu;
            int j1 = (closed_v && !sing_v0 && !sing_v1)
                     ? ((jj + 1) % nv_grid + j_start)
                     : (j + 1);
            size_t v00 = grid_idx(i, j), v10 = grid_idx(i1, j);
            size_t v01 = grid_idx(i, j1), v11 = grid_idx(i1, j1);
            if ((i + jj) % 2 == 0) {
                result.add_face({v00, v10, v11});
                result.add_face({v00, v11, v01});
            } else {
                result.add_face({v00, v10, v01});
                result.add_face({v10, v11, v01});
            }
        }

    // North pole fan
    if (sing_v1) {
        int j_last = j_end - 1;
        for (int i = 0; i < nu_faces; ++i) {
            int i1 = (i + 1) % nu;
            result.add_face({grid_idx(i, j_last), grid_idx(i1, j_last), north_pole});
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
