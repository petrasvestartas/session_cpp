#include "trimmedsurface.h"
#include "closest.h"
#include "primitives.h"
#include "trimesh_delaunay.h"
#include "trimesh_grid.h"
#include "triangulation_2d.h"
#include "fmt/core.h"
#include <fstream>
#include <set>
#include "trimmedsurface.pb.h"

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors
///////////////////////////////////////////////////////////////////////////////////////////

TrimmedSurface::TrimmedSurface() {}

TrimmedSurface::TrimmedSurface(const TrimmedSurface& other) {
    deep_copy_from(other);
}

TrimmedSurface& TrimmedSurface::operator=(const TrimmedSurface& other) {
    if (this != &other) deep_copy_from(other);
    return *this;
}

bool TrimmedSurface::operator==(const TrimmedSurface& other) const {
    if (name != other.name) return false;
    if (width != other.width) return false;
    if (surfacecolor != other.surfacecolor) return false;
    if (xform != other.xform) return false;
    if (m_surface != other.m_surface) return false;
    return true;
}

bool TrimmedSurface::operator!=(const TrimmedSurface& other) const {
    return !(*this == other);
}

TrimmedSurface::~TrimmedSurface() {}

void TrimmedSurface::deep_copy_from(const TrimmedSurface& src) {
    guid = ::guid();
    name = src.name;
    width = src.width;
    surfacecolor = src.surfacecolor;
    xform = src.xform;
    m_surface = src.m_surface;
    m_outer_loop = src.m_outer_loop;
    m_inner_loops = src.m_inner_loops;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Static Factory Methods
///////////////////////////////////////////////////////////////////////////////////////////

TrimmedSurface TrimmedSurface::create(const NurbsSurface& surface, const NurbsCurve& outer_loop) {
    TrimmedSurface ts;
    ts.m_surface = surface;
    ts.m_outer_loop = outer_loop;
    return ts;
}

TrimmedSurface TrimmedSurface::create_planar(const NurbsCurve& boundary) {
    NurbsSurface srf = Primitives::create_planar(boundary);
    if (!srf.is_valid()) return TrimmedSurface();

    Point p00 = srf.get_cv(0, 0);
    Point p10 = srf.get_cv(1, 0);
    Point p01 = srf.get_cv(0, 1);
    Vector u_axis(p10[0]-p00[0], p10[1]-p00[1], p10[2]-p00[2]);
    Vector v_axis(p01[0]-p00[0], p01[1]-p00[1], p01[2]-p00[2]);
    double u_len2 = u_axis[0]*u_axis[0] + u_axis[1]*u_axis[1] + u_axis[2]*u_axis[2];
    double v_len2 = v_axis[0]*v_axis[0] + v_axis[1]*v_axis[1] + v_axis[2]*v_axis[2];
    if (u_len2 < 1e-28 || v_len2 < 1e-28) return TrimmedSurface();

    auto project_to_uv = [&](const Point& pt) -> Point {
        double dx = pt[0]-p00[0], dy = pt[1]-p00[1], dz = pt[2]-p00[2];
        double nu = (dx*u_axis[0] + dy*u_axis[1] + dz*u_axis[2]) / u_len2;
        double nv = (dx*v_axis[0] + dy*v_axis[1] + dz*v_axis[2]) / v_len2;
        return Point(nu, nv, 0.0);
    };

    std::vector<Point> uv_pts;
    if (boundary.degree() <= 1) {
        for (int i = 0; i < boundary.cv_count(); ++i)
            uv_pts.push_back(project_to_uv(boundary.get_cv(i)));
    } else {
        auto spans = boundary.get_span_vector();
        for (size_t si = 0; si + 1 < spans.size(); ++si) {
            int n_sub = 10;
            for (int k = 0; k <= n_sub; ++k) {
                double t = spans[si] + (spans[si+1] - spans[si]) * k / n_sub;
                Point uv = project_to_uv(boundary.point_at(t));
                if (uv_pts.empty() || (uv[0]-uv_pts.back()[0])*(uv[0]-uv_pts.back()[0]) +
                    (uv[1]-uv_pts.back()[1])*(uv[1]-uv_pts.back()[1]) > 1e-24)
                    uv_pts.push_back(uv);
            }
        }
    }

    TrimmedSurface ts;
    ts.m_surface = srf;
    if (uv_pts.size() >= 3)
        ts.m_outer_loop = NurbsCurve::create(false, 1, uv_pts);
    return ts;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////////////////

NurbsSurface TrimmedSurface::surface() const { return m_surface; }
NurbsCurve TrimmedSurface::get_outer_loop() const { return m_outer_loop; }
void TrimmedSurface::set_outer_loop(const NurbsCurve& loop) { m_outer_loop = loop; }
bool TrimmedSurface::is_trimmed() const { return m_outer_loop.is_valid(); }
bool TrimmedSurface::is_valid() const { return m_surface.is_valid(); }

///////////////////////////////////////////////////////////////////////////////////////////
// Inner Loops
///////////////////////////////////////////////////////////////////////////////////////////

void TrimmedSurface::add_inner_loop(const NurbsCurve& loop_2d) {
    m_inner_loops.push_back(loop_2d);
}

void TrimmedSurface::add_hole(const NurbsCurve& curve_3d) {
    auto dom = curve_3d.domain();
    auto sdom_u = m_surface.domain(0);
    auto sdom_v = m_surface.domain(1);
    double range_u = sdom_u.second - sdom_u.first;
    double range_v = sdom_v.second - sdom_v.first;
    int n_samples = std::max(curve_3d.cv_count() * 4, 32);
    std::vector<Point> uv_pts;
    for (int i = 0; i < n_samples; ++i) {
        double t = dom.first + (dom.second - dom.first) * i / n_samples;
        Point pt3d = curve_3d.point_at(t);
        auto [u, v, dist] = Closest::surface_point(m_surface, pt3d);
        double nu = (u - sdom_u.first) / range_u;
        double nv = (v - sdom_v.first) / range_v;
        uv_pts.push_back(Point(nu, nv, 0.0));
    }
    if (uv_pts.size() >= 3)
        m_inner_loops.push_back(NurbsCurve::create(true, 1, uv_pts));
}

void TrimmedSurface::add_holes(const std::vector<NurbsCurve>& curves_3d) {
    for (const auto& crv : curves_3d) add_hole(crv);
}

NurbsCurve TrimmedSurface::get_inner_loop(int index) const { return m_inner_loops[index]; }
int TrimmedSurface::inner_loop_count() const { return static_cast<int>(m_inner_loops.size()); }
void TrimmedSurface::clear_inner_loops() { m_inner_loops.clear(); }

///////////////////////////////////////////////////////////////////////////////////////////
// Evaluation
///////////////////////////////////////////////////////////////////////////////////////////

Point TrimmedSurface::point_at(double u, double v) const { return m_surface.point_at(u, v); }
Vector TrimmedSurface::normal_at(double u, double v) const { return m_surface.normal_at(u, v); }

///////////////////////////////////////////////////////////////////////////////////////////
// Meshing
///////////////////////////////////////////////////////////////////////////////////////////

Mesh TrimmedSurface::mesh() const {
    if (!is_trimmed()) return m_surface.mesh();

    bool planar = m_surface.is_planar();

    // Planar: control points only (no subdivision) — minimal CDT triangulation
    // Non-planar: curvature-adaptive sampling to capture boundary shape
    auto disc_loop = [&](const NurbsCurve& crv) -> std::vector<Point> {
        std::vector<Point> pts;
        if (crv.degree() <= 1 && !crv.is_rational()) {
            for (int i = 0; i < crv.cv_count(); ++i) pts.push_back(crv.get_cv(i));
        } else {
            int n = std::max(crv.cv_count() * 4, 16);
            auto [sampled, params] = crv.divide_by_count(n);
            pts = sampled;
        }
        while (pts.size() > 1) {
            double dx = pts.front()[0] - pts.back()[0];
            double dy = pts.front()[1] - pts.back()[1];
            if (dx*dx + dy*dy < 1e-20) pts.pop_back();
            else break;
        }
        return pts;
    };

    auto outer_uv = disc_loop(m_outer_loop);
    std::vector<std::vector<Point>> hole_uvs;
    for (const auto& inner : m_inner_loops)
        hole_uvs.push_back(disc_loop(inner));
    if (outer_uv.size() < 3) return m_surface.mesh();

    // UV bounding box from outer loop
    double bb_umin = 1e30, bb_vmin = 1e30, bb_umax = -1e30, bb_vmax = -1e30;
    for (const auto& p : outer_uv) {
        if (p[0] < bb_umin) bb_umin = p[0];
        if (p[1] < bb_vmin) bb_vmin = p[1];
        if (p[0] > bb_umax) bb_umax = p[0];
        if (p[1] > bb_vmax) bb_vmax = p[1];
    }

    // Flat coords for point_in_polygon_2d
    auto to_flat = [](const std::vector<Point>& pts) -> std::vector<double> {
        std::vector<double> c;
        c.reserve(pts.size() * 2);
        for (const auto& p : pts) { c.push_back(p[0]); c.push_back(p[1]); }
        return c;
    };
    auto outer_coords = to_flat(outer_uv);
    std::vector<std::vector<double>> hole_coords;
    for (const auto& hp : hole_uvs) hole_coords.push_back(to_flat(hp));

    auto inside_trim = [&](double u, double v) -> bool {
        if (!Triangulation2D::point_in_polygon_2d(u, v, outer_coords)) return false;
        for (const auto& hc : hole_coords)
            if (Triangulation2D::point_in_polygon_2d(u, v, hc)) return false;
        return true;
    };

    // Create CDT
    Delaunay2D dt(bb_umin, bb_vmin, bb_umax, bb_vmax);

    // Insert boundary vertices + constrain edges
    auto insert_loop = [&](const std::vector<Point>& pts) {
        std::vector<int> vis;
        vis.reserve(pts.size());
        for (const auto& p : pts) vis.push_back(dt.insert(p[0], p[1]));
        for (size_t i = 0; i < vis.size(); ++i) {
            size_t j = (i + 1) % vis.size();
            if (vis[i] >= 0 && vis[j] >= 0 && vis[i] != vis[j])
                dt.insert_constraint(vis[i], vis[j]);
        }
    };
    insert_loop(outer_uv);
    for (const auto& hp : hole_uvs) insert_loop(hp);

    // Interior Steiner points — non-planar only (span-adaptive grid filtered by trim)
    // Planar uses control-point boundary only: CDT produces minimal triangulation directly
    if (!planar) {
        // Curvature-adaptive span subdivision
        auto usp = m_surface.get_span_vector(0);
        auto vsp = m_surface.get_span_vector(1);
        int deg_u = m_surface.degree(0), deg_v = m_surface.degree(1);
        int ns_u = (int)usp.size() - 1, ns_v = (int)vsp.size() - 1;

        // 3D bbox diagonal for chord tolerance
        double bmin[3] = {1e30, 1e30, 1e30}, bmax[3] = {-1e30, -1e30, -1e30};
        for (int i = 0; i < m_surface.cv_count(0); ++i)
            for (int j = 0; j < m_surface.cv_count(1); ++j) {
                Point p = m_surface.get_cv(i, j);
                for (int k = 0; k < 3; ++k) {
                    if (p[k] < bmin[k]) bmin[k] = p[k];
                    if (p[k] > bmax[k]) bmax[k] = p[k];
                }
            }
        double bbox_diag = std::sqrt((bmax[0]-bmin[0])*(bmax[0]-bmin[0])+
            (bmax[1]-bmin[1])*(bmax[1]-bmin[1])+(bmax[2]-bmin[2])*(bmax[2]-bmin[2]));
        double max_angle_deg = 20.0;

        // Edge length limit
        Point pe00 = m_surface.point_at(usp.front(), vsp.front());
        Point pe10 = m_surface.point_at(usp.back(), vsp.front());
        Point pe01 = m_surface.point_at(usp.front(), vsp.back());
        double edx1 = pe10[0]-pe00[0], edy1 = pe10[1]-pe00[1], edz1 = pe10[2]-pe00[2];
        double edx2 = pe01[0]-pe00[0], edy2 = pe01[1]-pe00[1], edz2 = pe01[2]-pe00[2];
        double max_dim = std::max(std::sqrt(edx1*edx1+edy1*edy1+edz1*edz1),
                                  std::sqrt(edx2*edx2+edy2*edy2+edz2*edz2));
        double max_edge_len = (max_dim > 1e-10) ? max_dim / 10.0 : 0.0;

        auto span_subs_fn = [&](int dir, const std::vector<double>& sp,
                                const std::vector<double>& osp) -> std::vector<int> {
            int n = (int)sp.size() - 1;
            std::vector<int> subs(n, 1);
            int n_other = (int)osp.size() - 1;
            std::vector<double> s_pos(n_other);
            for (int k = 0; k < n_other; ++k)
                s_pos[k] = (osp[k] + osp[k + 1]) * 0.5;
            int degree_dir = (dir == 0) ? deg_u : deg_v;
            for (int i = 0; i < n; ++i) {
                double t0 = sp[i], t1 = sp[i + 1];
                if (degree_dir > 1) {
                    double ma = 0.0;
                    for (int si = 0; si < n_other; ++si) {
                        double s = s_pos[si];
                        double pnx = 0, pny = 0, pnz = 0, ta = 0.0;
                        for (int k = 0; k <= 4; ++k) {
                            double t = t0 + k * (t1 - t0) / 4.0;
                            Vector nrm = (dir == 0) ? m_surface.normal_at(t, s)
                                                    : m_surface.normal_at(s, t);
                            if (k > 0) {
                                double d = pnx*nrm[0]+pny*nrm[1]+pnz*nrm[2];
                                d = std::max(-1.0, std::min(1.0, d));
                                ta += std::acos(d) * 180.0 / Tolerance::PI;
                            }
                            pnx = nrm[0]; pny = nrm[1]; pnz = nrm[2];
                        }
                        if (ta > ma) ma = ta;
                    }
                    subs[i] = std::max(1, std::min((int)std::ceil(ma / max_angle_deg), 24));
                }
                {
                    double chord_tol = bbox_diag * 0.005;
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
                            double lx = px0+frac*(px1-px0), ly = py0+frac*(py1-py0), lz = pz0+frac*(pz1-pz0);
                            double ddx = pmx-lx, ddy = pmy-ly, ddz = pmz-lz;
                            double dev = std::sqrt(ddx*ddx+ddy*ddy+ddz*ddz);
                            if (dev > max_dev) max_dev = dev;
                        }
                    }
                    if (max_dev > chord_tol) {
                        int cs = std::max(2, (int)std::ceil(std::sqrt(max_dev / chord_tol)));
                        subs[i] = std::max(subs[i], std::min(cs, 24));
                    }
                }
                if (max_edge_len > 0) {
                    double s_mid = (osp.front() + osp.back()) * 0.5;
                    double px0, py0, pz0, px1, py1, pz1;
                    if (dir == 0) {
                        m_surface.point_at(t0, s_mid, px0, py0, pz0);
                        m_surface.point_at(t1, s_mid, px1, py1, pz1);
                    } else {
                        m_surface.point_at(s_mid, t0, px0, py0, pz0);
                        m_surface.point_at(s_mid, t1, px1, py1, pz1);
                    }
                    double sl = std::sqrt((px1-px0)*(px1-px0)+(py1-py0)*(py1-py0)+(pz1-pz0)*(pz1-pz0));
                    int es = std::max(1, (int)std::ceil(sl / max_edge_len));
                    subs[i] = std::max(subs[i], std::min(es, 64));
                }
                if (degree_dir > 1) subs[i] = std::max(subs[i], 2);
            }
            return subs;
        };

        auto u_subs = span_subs_fn(0, usp, vsp);
        auto v_subs = span_subs_fn(1, vsp, usp);

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

        // Insert grid points inside trim region
        for (size_t i = 0; i < us.size(); ++i)
            for (size_t j = 0; j < vs.size(); ++j)
                if (inside_trim(us[i], vs[j])) dt.insert(us[i], vs[j]);
    }

    // Cleanup super-triangle
    dt.cleanup();

    // Remove exterior triangles
    for (auto& tri : dt.triangles) {
        if (!tri.alive) continue;
        double cu = (dt.vertices[tri.v[0]].x + dt.vertices[tri.v[1]].x + dt.vertices[tri.v[2]].x) / 3.0;
        double cv = (dt.vertices[tri.v[0]].y + dt.vertices[tri.v[1]].y + dt.vertices[tri.v[2]].y) / 3.0;
        if (!inside_trim(cu, cv)) tri.alive = false;
    }

    // Build output mesh
    auto tris = dt.get_triangles();
    if (tris.empty()) return m_surface.mesh();

    Mesh result;
    std::vector<size_t> vert_map(dt.vertices.size(), SIZE_MAX);
    for (const auto& tri : tris)
        for (int k = 0; k < 3; ++k) {
            int vi = tri[k];
            if (vert_map[vi] != SIZE_MAX) continue;
            vert_map[vi] = result.add_vertex(m_surface.point_at(dt.vertices[vi].x, dt.vertices[vi].y));
        }
    for (const auto& tri : tris) {
        size_t v0 = vert_map[tri[0]], v1 = vert_map[tri[1]], v2 = vert_map[tri[2]];
        if (v0 == v1 || v1 == v2 || v2 == v0) continue;
        result.add_face({v0, v1, v2});
    }

    // Normals
    if (planar) {
        auto du = m_surface.domain(0), dv = m_surface.domain(1);
        Vector nrm = m_surface.normal_at((du.first+du.second)/2, (dv.first+dv.second)/2);
        for (auto& [vk, vd] : result.vertex) vd.set_normal(nrm[0], nrm[1], nrm[2]);
    } else {
        for (size_t vi = 0; vi < vert_map.size(); ++vi) {
            if (vert_map[vi] == SIZE_MAX) continue;
            Vector nrm = m_surface.normal_at(dt.vertices[vi].x, dt.vertices[vi].y);
            result.vertex[vert_map[vi]].set_normal(nrm[0], nrm[1], nrm[2]);
        }
    }
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void TrimmedSurface::transform() {
    m_surface.xform = xform;
    m_surface.transform();
    xform = Xform::identity();
}

TrimmedSurface TrimmedSurface::transformed() const {
    TrimmedSurface ts = *this;
    ts.transform();
    return ts;
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

nlohmann::ordered_json TrimmedSurface::jsondump() const {
    nlohmann::ordered_json j;
    j["guid"] = guid;
    j["inner_loops"] = nlohmann::ordered_json::array();
    for (const auto& loop : m_inner_loops)
        j["inner_loops"].push_back(loop.jsondump());
    j["name"] = name;
    if (m_outer_loop.is_valid())
        j["outer_loop"] = m_outer_loop.jsondump();
    j["surface"] = m_surface.jsondump();
    j["surfacecolor"] = surfacecolor.jsondump();
    j["type"] = "TrimmedSurface";
    j["width"] = width;
    j["xform"] = xform.jsondump();
    return j;
}

TrimmedSurface TrimmedSurface::jsonload(const nlohmann::json& data) {
    TrimmedSurface ts;
    if (data.contains("guid")) ts.guid = data["guid"];
    if (data.contains("name")) ts.name = data["name"];
    if (data.contains("width")) ts.width = data["width"];
    if (data.contains("surfacecolor")) ts.surfacecolor = Color::jsonload(data["surfacecolor"]);
    if (data.contains("xform")) ts.xform = Xform::jsonload(data["xform"]);
    if (data.contains("surface")) ts.m_surface = NurbsSurface::jsonload(data["surface"]);
    if (data.contains("outer_loop")) ts.m_outer_loop = NurbsCurve::jsonload(data["outer_loop"]);
    if (data.contains("inner_loops")) {
        for (const auto& loop_data : data["inner_loops"])
            ts.m_inner_loops.push_back(NurbsCurve::jsonload(loop_data));
    }
    return ts;
}

std::string TrimmedSurface::json_dumps() const { return jsondump().dump(); }
TrimmedSurface TrimmedSurface::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void TrimmedSurface::json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

TrimmedSurface TrimmedSurface::json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf Serialization
///////////////////////////////////////////////////////////////////////////////////////////

std::string TrimmedSurface::pb_dumps() const {
    session_proto::TrimmedSurface proto;
    proto.set_guid(guid);
    proto.set_name(name);
    proto.set_width(width);

    // Surface
    std::string srf_data = m_surface.pb_dumps();
    auto* srf_proto = proto.mutable_surface();
    srf_proto->ParseFromString(srf_data);

    // Outer loop
    if (is_trimmed()) {
        std::string loop_data = m_outer_loop.pb_dumps();
        auto* ol = proto.mutable_outer_loop();
        ol->ParseFromString(loop_data);
    }

    // Inner loops
    for (const auto& inner : m_inner_loops) {
        std::string loop_data = inner.pb_dumps();
        auto* il = proto.add_inner_loops();
        il->ParseFromString(loop_data);
    }

    // Color
    auto* color_proto = proto.mutable_surfacecolor();
    color_proto->set_name(surfacecolor.name);
    color_proto->set_r(surfacecolor.r);
    color_proto->set_g(surfacecolor.g);
    color_proto->set_b(surfacecolor.b);
    color_proto->set_a(surfacecolor.a);

    // Transform
    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_guid(xform.guid);
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i)
        xform_proto->add_matrix(xform.m[i]);

    return proto.SerializeAsString();
}

TrimmedSurface TrimmedSurface::pb_loads(const std::string& data) {
    session_proto::TrimmedSurface proto;
    proto.ParseFromString(data);

    TrimmedSurface ts;
    ts.guid = proto.guid();
    ts.name = proto.name();
    ts.width = proto.width();

    // Surface
    if (proto.has_surface()) {
        std::string srf_data = proto.surface().SerializeAsString();
        ts.m_surface = NurbsSurface::pb_loads(srf_data);
    }

    // Outer loop
    if (proto.has_outer_loop()) {
        std::string loop_data = proto.outer_loop().SerializeAsString();
        ts.m_outer_loop = NurbsCurve::pb_loads(loop_data);
    }

    // Inner loops
    for (int i = 0; i < proto.inner_loops_size(); ++i) {
        std::string loop_data = proto.inner_loops(i).SerializeAsString();
        ts.m_inner_loops.push_back(NurbsCurve::pb_loads(loop_data));
    }

    // Color
    const auto& color_proto = proto.surfacecolor();
    ts.surfacecolor.name = color_proto.name();
    ts.surfacecolor.r = color_proto.r();
    ts.surfacecolor.g = color_proto.g();
    ts.surfacecolor.b = color_proto.b();
    ts.surfacecolor.a = color_proto.a();

    // Transform
    const auto& xform_proto = proto.xform();
    ts.xform.guid = xform_proto.guid();
    ts.xform.name = xform_proto.name();
    for (int i = 0; i < 16 && i < xform_proto.matrix_size(); ++i)
        ts.xform.m[i] = xform_proto.matrix(i);

    return ts;
}

void TrimmedSurface::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

TrimmedSurface TrimmedSurface::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
    return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// String Representation
///////////////////////////////////////////////////////////////////////////////////////////

std::string TrimmedSurface::str() const {
    return fmt::format("TrimmedSurface(name={}, trimmed={}, holes={})",
                       name, is_trimmed() ? "true" : "false", inner_loop_count());
}

std::string TrimmedSurface::repr() const {
    return fmt::format("TrimmedSurface(\n  name={},\n  trimmed={},\n  holes={},\n  surface={}\n)",
                       name, is_trimmed() ? "true" : "false", inner_loop_count(), m_surface.str());
}

std::ostream& operator<<(std::ostream& os, const TrimmedSurface& ts) {
    os << ts.str();
    return os;
}

} // namespace session_cpp
