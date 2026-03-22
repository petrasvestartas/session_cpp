#include "remesh_nurbssurface_grid.h"
#include "tolerance.h"
#include <cmath>
#include <algorithm>

namespace session_cpp {

static constexpr double MAX_ANGLE = 20.0;

Mesh remesh_nurbssurface_grid(const NurbsSurface& s, int max_u, int max_v) {
    std::vector<double> usp = s.get_span_vector(0);
    std::vector<double> vsp = s.get_span_vector(1);
    int ns_u = (int)usp.size() - 1, ns_v = (int)vsp.size() - 1;

    int deg_u = s.degree(0), deg_v = s.degree(1);

    double minx = 1e30, miny = 1e30, minz = 1e30;
    double maxx = -1e30, maxy = -1e30, maxz = -1e30;
    for (int i = 0; i < s.cv_count(0); ++i) {
        for (int j = 0; j < s.cv_count(1); ++j) {
            Point p = s.get_cv(i, j);
            if (p[0] < minx) minx = p[0];
            if (p[1] < miny) miny = p[1];
            if (p[2] < minz) minz = p[2];
            if (p[0] > maxx) maxx = p[0];
            if (p[1] > maxy) maxy = p[1];
            if (p[2] > maxz) maxz = p[2];
        }
    }
    double dx = maxx - minx, dy = maxy - miny, dz = maxz - minz;
    double bbox_diag = std::sqrt(dx * dx + dy * dy + dz * dz);

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
                    double sv = s_positions[si];
                    double fn[3]={0,0,0}, ln[3]={0,0,0};
                    bool has_first = false;
                    for (int k = 0; k <= 4; ++k) {
                        double t = t0 + k * (t1 - t0) / 4.0;
                        Vector nrm = (dir == 0) ? s.normal_at(t, sv) : s.normal_at(sv, t);
                        double nx = nrm[0], ny = nrm[1], nz = nrm[2];
                        double len = std::sqrt(nx*nx + ny*ny + nz*nz);
                        if (len < 1e-10) continue;
                        nx/=len; ny/=len; nz/=len;
                        if (!has_first) { fn[0]=nx; fn[1]=ny; fn[2]=nz; has_first=true; }
                        ln[0]=nx; ln[1]=ny; ln[2]=nz;
                    }
                    double total_angle = 0.0;
                    if (has_first) {
                        double dot = fn[0]*ln[0] + fn[1]*ln[1] + fn[2]*ln[2];
                        total_angle = std::acos(std::max(-1.0, std::min(1.0, dot))) * 180.0 / Tolerance::PI;
                    }
                    if (total_angle > max_angle) max_angle = total_angle;
                }
                subs[i] = std::max(1, (int)std::ceil(max_angle / MAX_ANGLE));
            }

            // Direct chord-height deviation check
            {
                double chord_tol = bbox_diag * 0.005;
                double max_dev = 0.0;
                int nc = std::min(n_other, 3);
                for (int ci = 0; ci <= nc; ++ci) {
                    double sv = osp.front() + ci * (osp.back() - osp.front()) / std::max(nc, 1);
                    double px0, py0, pz0, px1, py1, pz1;
                    if (dir == 0) {
                        s.point_at(t0, sv, px0, py0, pz0);
                        s.point_at(t1, sv, px1, py1, pz1);
                    } else {
                        s.point_at(sv, t0, px0, py0, pz0);
                        s.point_at(sv, t1, px1, py1, pz1);
                    }
                    for (int k = 1; k <= 3; ++k) {
                        double frac = k / 4.0;
                        double tm = t0 + frac * (t1 - t0);
                        double pmx, pmy, pmz;
                        if (dir == 0) s.point_at(tm, sv, pmx, pmy, pmz);
                        else          s.point_at(sv, tm, pmx, pmy, pmz);
                        double lx = px0 + frac * (px1 - px0);
                        double ly = py0 + frac * (py1 - py0);
                        double lz = pz0 + frac * (pz1 - pz0);
                        double ddx = pmx - lx, ddy = pmy - ly, ddz = pmz - lz;
                        double dev = std::sqrt(ddx*ddx + ddy*ddy + ddz*ddz);
                        if (dev > max_dev) max_dev = dev;
                    }
                }
                if (max_dev > chord_tol) {
                    int chord_subs = std::max(2, (int)std::ceil(std::sqrt(max_dev / chord_tol)));
                    subs[i] = std::max(subs[i], chord_subs);
                }
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
        for (int sv : u_subs) total_u += sv;
        for (int sv : v_subs) total_v += sv;
        total_u += 1; total_v += 1;
        double v_mid = (vsp.front() + vsp.back()) * 0.5;
        double u_mid = (usp.front() + usp.back()) * 0.5;
        double u_len = 0.0, v_len = 0.0;
        {
            double px0, py0, pz0;
            s.point_at(usp.front(), v_mid, px0, py0, pz0);
            int n_sample = std::max(total_u, 10);
            for (int i = 1; i <= n_sample; ++i) {
                double u = usp.front() + i * (usp.back() - usp.front()) / n_sample;
                double px1, py1, pz1;
                s.point_at(u, v_mid, px1, py1, pz1);
                u_len += std::sqrt((px1-px0)*(px1-px0)+(py1-py0)*(py1-py0)+(pz1-pz0)*(pz1-pz0));
                px0 = px1; py0 = py1; pz0 = pz1;
            }
        }
        {
            double px0, py0, pz0;
            s.point_at(u_mid, vsp.front(), px0, py0, pz0);
            int n_sample = std::max(total_v, 10);
            for (int i = 1; i <= n_sample; ++i) {
                double v = vsp.front() + i * (vsp.back() - vsp.front()) / n_sample;
                double px1, py1, pz1;
                s.point_at(u_mid, v, px1, py1, pz1);
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
                for (int& sv : u_subs) sv = (int)std::ceil(sv * scale);
            } else if (ratio < 0.5 && deg_v > 1) {
                double scale = std::sqrt(1.0 / ratio);
                for (int& sv : v_subs) sv = (int)std::ceil(sv * scale);
            }
        }
    }

    // Bilinear twist check (skip for singular surfaces — fan triangulation handles those)
    if (deg_u == 1 && deg_v == 1 && !s.is_singular(0) && !s.is_singular(2)) {
        double chord_tol = (bbox_diag > 0) ? bbox_diag * 0.005 : 1e-6;
        double max_twist = 0.0;
        for (int i = 0; i < ns_u; ++i)
            for (int j = 0; j < ns_v; ++j) {
                double u0 = usp[i], u1 = usp[i+1];
                double v0 = vsp[j], v1 = vsp[j+1];
                double pmx, pmy, pmz;
                s.point_at((u0+u1)*0.5, (v0+v1)*0.5, pmx, pmy, pmz);
                double p00x, p00y, p00z, p11x, p11y, p11z;
                s.point_at(u0, v0, p00x, p00y, p00z);
                s.point_at(u1, v1, p11x, p11y, p11z);
                double mx = (p00x+p11x)*0.5, my = (p00y+p11y)*0.5, mz = (p00z+p11z)*0.5;
                double ddx = pmx-mx, ddy = pmy-my, ddz = pmz-mz;
                double twist = std::sqrt(ddx*ddx+ddy*ddy+ddz*ddz);
                if (twist > max_twist) max_twist = twist;
            }
        if (max_twist > chord_tol) {
            int twist_subs = std::max(4, (int)std::ceil(2.0 * std::sqrt(max_twist / chord_tol)));
            for (int& sv : u_subs) sv = std::max(sv, twist_subs);
            for (int& sv : v_subs) sv = std::max(sv, twist_subs);
        }
    }

    bool closed_u = s.is_closed(0);
    bool closed_v = s.is_closed(1);

    // Ensure odd total subdivisions for closed directions (seamless checkerboard triangulation)
    if (closed_u && max_u == 0) {
        int total = 0; for (int sv : u_subs) total += sv;
        if (total % 2 == 0) *std::max_element(u_subs.begin(), u_subs.end()) += 1;
    }
    if (closed_v && max_v == 0) {
        int total = 0; for (int sv : v_subs) total += sv;
        if (total % 2 == 0) *std::max_element(v_subs.begin(), v_subs.end()) += 1;
    }

    // Arc-length parameterization: sample dense curve, redistribute n points evenly by 3D length
    auto arclen_params = [&](int n, const std::vector<double>& sp, double fixed) -> std::vector<double> {
        int nsample = std::max(n * 20, 200);
        std::vector<double> st(nsample + 1);
        std::vector<double> sl(nsample + 1, 0.0);
        double px0, py0, pz0;
        bool is_u = (&sp == &usp);
        double t0 = sp.front();
        if (is_u) s.point_at(t0, fixed, px0, py0, pz0);
        else      s.point_at(fixed, t0, px0, py0, pz0);
        for (int k = 0; k <= nsample; ++k) {
            double t = sp.front() + k * (sp.back() - sp.front()) / nsample;
            st[k] = t;
            if (k > 0) {
                double px1, py1, pz1;
                if (is_u) s.point_at(t, fixed, px1, py1, pz1);
                else      s.point_at(fixed, t, px1, py1, pz1);
                double d = std::sqrt((px1-px0)*(px1-px0)+(py1-py0)*(py1-py0)+(pz1-pz0)*(pz1-pz0));
                sl[k] = sl[k-1] + d;
                px0 = px1; py0 = py1; pz0 = pz1;
            }
        }
        double total = sl[nsample];
        std::vector<double> params;
        params.push_back(sp.front());
        int j = 0;
        for (int i = 1; i < n - 1; ++i) {
            double target = total * i / (n - 1);
            while (j < nsample && sl[j] < target) ++j;
            double ta = st[j > 0 ? j-1 : 0], tb = st[j];
            double la = sl[j > 0 ? j-1 : 0], lb = sl[j];
            double frac = (lb > la) ? (target - la) / (lb - la) : 0.0;
            params.push_back(ta + frac * (tb - ta));
        }
        params.push_back(sp.back());
        return params;
    };

    // Build parameter arrays
    double v_mid = (vsp.front() + vsp.back()) * 0.5;
    double u_mid = (usp.front() + usp.back()) * 0.5;
    std::vector<double> us, vs;
    if (max_u > 0) {
        us = arclen_params(std::max(max_u, 2), usp, v_mid);
    } else {
        for (int i = 0; i < ns_u; ++i)
            for (int sv = 0; sv < u_subs[i]; ++sv)
                us.push_back(usp[i] + sv * (usp[i+1] - usp[i]) / u_subs[i]);
        us.push_back(usp.back());
    }
    if (max_v > 0) {
        vs = arclen_params(std::max(max_v, 2), vsp, u_mid);
    } else {
        for (int i = 0; i < ns_v; ++i)
            for (int sv = 0; sv < v_subs[i]; ++sv)
                vs.push_back(vsp[i] + sv * (vsp[i+1] - vsp[i]) / v_subs[i]);
        vs.push_back(vsp.back());
    }

    // For closed surfaces, ensure the wrapping gap is not disproportionately large.
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

    // Detect singular edges (collapsed to a single point)
    bool sing_v0 = s.is_singular(0); // south: v=vs[0]
    bool sing_v1 = s.is_singular(2); // north: v=vs[nv-1]
    int j_start = sing_v0 ? 1 : 0;
    int j_end = sing_v1 ? nv - 1 : nv;
    int nv_grid = j_end - j_start;

    Mesh result;
    size_t south_pole = 0, north_pole = 0;
    if (sing_v0) {
        double px, py, pz;
        s.point_at(us[0], vs[0], px, py, pz);
        south_pole = result.add_vertex(Point(px, py, pz));
        result.vertex[south_pole].attributes["u"] = us[0];
        result.vertex[south_pole].attributes["v"] = vs[0];
    }
    if (sing_v1) {
        double px, py, pz;
        s.point_at(us[0], vs[nv - 1], px, py, pz);
        north_pole = result.add_vertex(Point(px, py, pz));
        result.vertex[north_pole].attributes["u"] = us[0];
        result.vertex[north_pole].attributes["v"] = vs[nv - 1];
    }
    size_t grid_base = result.vertex.size();
    for (int i = 0; i < nu; ++i)
        for (int j = j_start; j < j_end; ++j) {
            double px, py, pz;
            s.point_at(us[i], vs[j], px, py, pz);
            size_t vk = result.add_vertex(Point(px, py, pz));
            result.vertex[vk].attributes["u"] = us[i];
            result.vertex[vk].attributes["v"] = vs[j];
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
        Point pos0 = result.vertex.at(vids[0]).position();
        Point pos1 = result.vertex.at(vids[1]).position();
        Point pos2 = result.vertex.at(vids[2]).position();
        double e1x = pos1[0]-pos0[0], e1y = pos1[1]-pos0[1], e1z = pos1[2]-pos0[2];
        double e2x = pos2[0]-pos0[0], e2y = pos2[1]-pos0[1], e2z = pos2[2]-pos0[2];
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

Mesh RemeshNurbsSurfaceGrid::from_u_v(const NurbsSurface& s, int max_u, int max_v) {
    return remesh_nurbssurface_grid(s, max_u, max_v);
}

} // namespace session_cpp
