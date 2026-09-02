#include "brep.h"
#include "nurbssurface_trimmed.h"
#include "remesh_nurbssurface_grid.h"
#include "primitives.h"
#include "plane.h"
#include "fmt/core.h"
#include "brep.pb.h"
#include <fstream>
#include <cmath>
#include <map>
#include <algorithm>

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Orientation
///////////////////////////////////////////////////////////////////////////////////////////

BRepOrientation brep_reverse(BRepOrientation o) {
    if (o == BRepOrientation::Forward) return BRepOrientation::Reversed;
    if (o == BRepOrientation::Reversed) return BRepOrientation::Forward;
    return o;
}

BRepOrientation brep_compose(BRepOrientation a, BRepOrientation b) {
    if (a == BRepOrientation::Internal || a == BRepOrientation::External) return a;
    if (a == BRepOrientation::Forward) return b;
    return brep_reverse(b);
}

static const char* orientation_to_str(BRepOrientation o) {
    switch (o) {
        case BRepOrientation::Forward: return "forward";
        case BRepOrientation::Reversed: return "reversed";
        case BRepOrientation::Internal: return "internal";
        case BRepOrientation::External: return "external";
    }
    return "forward";
}

static BRepOrientation orientation_from_str(const std::string& s) {
    if (s == "reversed") return BRepOrientation::Reversed;
    if (s == "internal") return BRepOrientation::Internal;
    if (s == "external") return BRepOrientation::External;
    return BRepOrientation::Forward;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Geometry helpers
///////////////////////////////////////////////////////////////////////////////////////////

static const BRepOrientation F = BRepOrientation::Forward;
static const BRepOrientation R = BRepOrientation::Reversed;

/// Bilinear planar patch: u runs p00 -> p10, v runs p00 -> p01, natural normal = u x v.
static NurbsSurface bilinear_patch(const Point& p00, const Point& p10, const Point& p01, const Point& p11) {
    NurbsSurface srf;
    srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
    srf.set_cv(0, 0, p00);
    srf.set_cv(1, 0, p10);
    srf.set_cv(0, 1, p01);
    srf.set_cv(1, 1, p11);
    return srf;
}

/// Straight pcurve from (u0, v0) to (u1, v1).
static NurbsCurve uv_line(double u0, double v0, double u1, double v1) {
    return NurbsCurve::create(false, 1, {Point(u0, v0, 0), Point(u1, v1, 0)});
}

/// Exact pcurve of a 3D curve lying on a bilinear planar patch: the affine image of its CVs.
static NurbsCurve project_to_patch(const NurbsCurve& crv, const NurbsSurface& srf) {
    Point p00 = srf.get_cv(0, 0), p10 = srf.get_cv(1, 0), p01 = srf.get_cv(0, 1);
    double eu[3] = {p10[0] - p00[0], p10[1] - p00[1], p10[2] - p00[2]};
    double ev[3] = {p01[0] - p00[0], p01[1] - p00[1], p01[2] - p00[2]};
    double eu2 = eu[0]*eu[0] + eu[1]*eu[1] + eu[2]*eu[2];
    double ev2 = ev[0]*ev[0] + ev[1]*ev[1] + ev[2]*ev[2];
    NurbsCurve c2(3, crv.is_rational(), crv.order(), crv.cv_count());
    for (int i = 0; i < crv.nurbsknot_count(); ++i) c2.set_nurbsknot(i, crv.nurbsknot(i));
    for (int i = 0; i < crv.cv_count(); ++i) {
        auto [wx, wy, wz, w] = crv.get_cv_4d(i);
        double x = wx / w, y = wy / w, z = wz / w;
        double dx = x - p00[0], dy = y - p00[1], dz = z - p00[2];
        double u = (dx*eu[0] + dy*eu[1] + dz*eu[2]) / eu2;
        double v = (dx*ev[0] + dy*ev[1] + dz*ev[2]) / ev2;
        if (crv.is_rational()) c2.set_cv_4d(i, u*w, v*w, 0.0, w);
        else c2.set_cv(i, Point(u, v, 0));
    }
    return c2;
}

/// Signed area of a closed pcurve's sampled polygon (positive = counter-clockwise).
static double uv_signed_area(const NurbsCurve& c2d) {
    auto [pts, params] = c2d.divide_by_count(std::max(c2d.cv_count() * 4, 16), true);
    double a = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i)
        a += pts[i][0] * pts[i+1][1] - pts[i+1][0] * pts[i][1];
    return 0.5 * a;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Static Factory Methods
///////////////////////////////////////////////////////////////////////////////////////////

namespace {
/// Shared builder for planar polygon faces whose vertices, edges (lo -> hi vertex) and
/// surfaces are made from a point table. The face order lists vertices counter-clockwise
/// seen from the outside, so the natural normal of the patch points outward (Forward face).
struct PolyFaceBuilder {
    BRep& b;
    std::map<std::pair<int, int>, int> edge_map;

    int edge(int v0, int v1) {
        int lo = std::min(v0, v1), hi = std::max(v0, v1);
        auto it = edge_map.find({lo, hi});
        if (it != edge_map.end()) return it->second;
        NurbsCurve line = NurbsCurve::create(false, 1, {b.m_vertices[lo].point, b.m_vertices[hi].point});
        int ei = b.add_edge(b.add_curve_3d(line), lo, hi);
        edge_map[{lo, hi}] = ei;
        return ei;
    }

    /// Face on `srf` bounded by the vertex cycle `vi`; returns the face index.
    int face(const NurbsSurface& srf, const std::vector<int>& vi, std::vector<BRepRef>* wire_out = nullptr) {
        int si = b.add_surface(srf);
        std::vector<BRepRef> refs;
        int n = (int)vi.size();
        for (int i = 0; i < n; ++i) {
            int va = vi[i], vb = vi[(i + 1) % n];
            int ei = edge(va, vb);
            b.add_pcurve(ei, si, b.add_curve_2d(project_to_patch(b.m_curves_3d[b.m_edges[ei].curve_3d_index], srf)));
            refs.push_back({ei, b.m_edges[ei].start_vertex == va ? F : R});
        }
        if (wire_out) { *wire_out = refs; return si; }
        return b.add_face(si, {{b.add_wire(refs), F}});
    }
};

/// Bilinear patch spanned by four vertex indices in face order (p00, p10, p11, p01).
NurbsSurface quad_patch(const BRep& b, const int fv[4]) {
    return bilinear_patch(b.m_vertices[fv[0]].point, b.m_vertices[fv[1]].point,
                          b.m_vertices[fv[3]].point, b.m_vertices[fv[2]].point);
}

const int BOX_FACES[6][4] = {
    {0, 3, 2, 1}, // bottom (z=-hz), normal -Z
    {4, 5, 6, 7}, // top (z=+hz), normal +Z
    {0, 1, 5, 4}, // front (y=-hy), normal -Y
    {1, 2, 6, 5}, // right (x=+hx), normal +X
    {2, 3, 7, 6}, // back (y=+hy), normal +Y
    {3, 0, 4, 7}, // left (x=-hx), normal -X
};

void box_corners(BRep& b, double sx, double sy, double sz) {
    double hx = sx * 0.5, hy = sy * 0.5, hz = sz * 0.5;
    b.add_vertex(Point(-hx, -hy, -hz));
    b.add_vertex(Point( hx, -hy, -hz));
    b.add_vertex(Point( hx,  hy, -hz));
    b.add_vertex(Point(-hx,  hy, -hz));
    b.add_vertex(Point(-hx, -hy,  hz));
    b.add_vertex(Point( hx, -hy,  hz));
    b.add_vertex(Point( hx,  hy,  hz));
    b.add_vertex(Point(-hx,  hy,  hz));
}

/// Planar cap at height z with natural normal +Z (up) or -Z (down), spanning [-r, r]^2.
NurbsSurface cap_patch(double r, double z, bool up) {
    return up ? bilinear_patch(Point(-r, -r, z), Point(r, -r, z), Point(-r, r, z), Point(r, r, z))
              : bilinear_patch(Point(-r, -r, z), Point(-r, r, z), Point(r, -r, z), Point(r, r, z));
}

/// Cap face bounded by one closed edge: outer wire counter-clockwise in the patch's UV.
int cap_face(BRep& b, const NurbsSurface& cap, int edge) {
    int si = b.add_surface(cap);
    NurbsCurve c2d = project_to_patch(b.m_curves_3d[b.m_edges[edge].curve_3d_index], cap);
    BRepOrientation o = uv_signed_area(c2d) > 0.0 ? F : R;
    b.add_pcurve(edge, si, b.add_curve_2d(c2d));
    return b.add_face(si, {{b.add_wire({{edge, o}}), F}});
}

/// Periodic body face (cylinder / cone / bore): seam from v0 to v1 at u0 == u1, bottom ring
/// forward at v0, top ring (or degenerated apex) reversed at v1.
int body_face(BRep& b, int si, int e_bot, int e_seam, int e_top) {
    auto [u0, u1] = b.m_surfaces[si].domain(0);
    auto [v0, v1] = b.m_surfaces[si].domain(1);
    b.add_pcurve(e_bot, si, b.add_curve_2d(uv_line(u0, v0, u1, v0)));
    b.add_pcurve(e_top, si, b.add_curve_2d(uv_line(u0, v1, u1, v1)));
    b.add_pcurve(e_seam, si, b.add_curve_2d(uv_line(u1, v0, u1, v1)), b.add_curve_2d(uv_line(u0, v0, u0, v1)));
    return b.add_face(si, {{b.add_wire({{e_bot, F}, {e_seam, F}, {e_top, R}, {e_seam, R}}), F}});
}
} // namespace

BRep BRep::create_box(double sx, double sy, double sz) {
    BRep b;
    b.name = "box";
    box_corners(b, sx, sy, sz);
    PolyFaceBuilder pb{b, {}};
    std::vector<BRepRef> faces;
    for (int fi = 0; fi < 6; ++fi) {
        const int* fv = BOX_FACES[fi];
        faces.push_back({pb.face(quad_patch(b, fv), {fv[0], fv[1], fv[2], fv[3]}), F});
    }
    b.add_solid({{b.add_shell(faces), F}});
    return b;
}

BRep BRep::create_cylinder(double radius, double height) {
    BRep b;
    b.name = "cylinder";
    NurbsSurface body = Primitives::cylinder_surface(0, 0, 0, radius, height);
    Point p_bot = body.point_at_corner(0, 0);
    Point p_top = body.point_at_corner(0, 1);
    int v_bot = b.add_vertex(p_bot);
    int v_top = b.add_vertex(p_top);
    int e_bot = b.add_edge(b.add_curve_3d(Primitives::circle(0, 0, 0, radius)), v_bot, v_bot);
    int e_top = b.add_edge(b.add_curve_3d(Primitives::circle(0, 0, height, radius)), v_top, v_top);
    int e_seam = b.add_edge(b.add_curve_3d(NurbsCurve::create(false, 1, {p_bot, p_top})), v_bot, v_top);
    int f_body = body_face(b, b.add_surface(body), e_bot, e_seam, e_top);
    int f_bot = cap_face(b, cap_patch(radius, 0, false), e_bot);
    int f_top = cap_face(b, cap_patch(radius, height, true), e_top);
    b.add_solid({{b.add_shell({{f_body, F}, {f_bot, F}, {f_top, F}}), F}});
    return b;
}

BRep BRep::create_sphere(double radius) {
    BRep b;
    b.name = "sphere";
    NurbsSurface srf = Primitives::sphere_surface(0, 0, 0, radius);
    auto [u0, u1] = srf.domain(0);
    auto [v0, v1] = srf.domain(1);
    int v_s = b.add_vertex(Point(0, 0, -radius));
    int v_n = b.add_vertex(Point(0, 0, radius));
    int e_seam = b.add_edge(b.add_curve_3d(srf.iso_curve(1, u0)), v_s, v_n);
    int e_south = b.add_edge(-1, v_s, v_s);
    int e_north = b.add_edge(-1, v_n, v_n);
    int si = b.add_surface(srf);
    b.add_pcurve(e_south, si, b.add_curve_2d(uv_line(u0, v0, u1, v0)));
    b.add_pcurve(e_north, si, b.add_curve_2d(uv_line(u0, v1, u1, v1)));
    b.add_pcurve(e_seam, si, b.add_curve_2d(uv_line(u1, v0, u1, v1)), b.add_curve_2d(uv_line(u0, v0, u0, v1)));
    int fi = b.add_face(si, {{b.add_wire({{e_south, F}, {e_seam, F}, {e_north, R}, {e_seam, R}}), F}});
    b.add_solid({{b.add_shell({{fi, F}}), F}});
    return b;
}

BRep BRep::create_cone(double radius, double height) {
    BRep b;
    b.name = "cone";
    NurbsSurface body = Primitives::cone_surface(0, 0, 0, radius, height);
    Point p_base = body.point_at_corner(0, 0);
    Point p_apex(0, 0, height);
    int v_base = b.add_vertex(p_base);
    int v_apex = b.add_vertex(p_apex);
    int e_base = b.add_edge(b.add_curve_3d(Primitives::circle(0, 0, 0, radius)), v_base, v_base);
    int e_seam = b.add_edge(b.add_curve_3d(NurbsCurve::create(false, 1, {p_base, p_apex})), v_base, v_apex);
    int e_apex = b.add_edge(-1, v_apex, v_apex);
    int f_body = body_face(b, b.add_surface(body), e_base, e_seam, e_apex);
    int f_base = cap_face(b, cap_patch(radius, 0, false), e_base);
    b.add_solid({{b.add_shell({{f_body, F}, {f_base, F}}), F}});
    return b;
}

BRep BRep::create_pyramid(double base, double height) {
    BRep b;
    b.name = "pyramid";
    double h = base * 0.5;
    b.add_vertex(Point(-h, -h, 0.0));
    b.add_vertex(Point( h, -h, 0.0));
    b.add_vertex(Point( h,  h, 0.0));
    b.add_vertex(Point(-h,  h, 0.0));
    int v_apex = b.add_vertex(Point(0.0, 0.0, height));
    PolyFaceBuilder pb{b, {}};
    std::vector<BRepRef> faces;
    const int fv[4] = {0, 3, 2, 1};
    faces.push_back({pb.face(quad_patch(b, fv), {0, 3, 2, 1}), F});
    for (int i = 0; i < 4; ++i) {
        int a = i, c = (i + 1) % 4;
        NurbsSurface srf = bilinear_patch(b.m_vertices[a].point, b.m_vertices[c].point, b.m_vertices[v_apex].point, b.m_vertices[v_apex].point);
        int si = b.add_surface(srf);
        int e_ac = pb.edge(a, c), e_c = pb.edge(c, v_apex), e_a = pb.edge(a, v_apex);
        int e_deg = b.add_edge(-1, v_apex, v_apex);
        bool ac_fwd = b.m_edges[e_ac].start_vertex == a;
        b.add_pcurve(e_ac, si, b.add_curve_2d(ac_fwd ? uv_line(0, 0, 1, 0) : uv_line(1, 0, 0, 0)));
        b.add_pcurve(e_c, si, b.add_curve_2d(uv_line(1, 0, 1, 1)));
        b.add_pcurve(e_deg, si, b.add_curve_2d(uv_line(1, 1, 0, 1)));
        b.add_pcurve(e_a, si, b.add_curve_2d(uv_line(0, 0, 0, 1)));
        faces.push_back({b.add_face(si, {{b.add_wire({{e_ac, ac_fwd ? F : R}, {e_c, F}, {e_deg, F}, {e_a, R}}), F}}), F});
    }
    b.add_solid({{b.add_shell(faces), F}});
    return b;
}

BRep BRep::create_torus(double major_radius, double minor_radius) {
    BRep b;
    b.name = "torus";
    NurbsSurface srf = Primitives::torus_surface(0, 0, 0, major_radius, minor_radius);
    auto [u0, u1] = srf.domain(0);
    auto [v0, v1] = srf.domain(1);
    int v = b.add_vertex(srf.point_at_corner(0, 0));
    int e_u = b.add_edge(b.add_curve_3d(srf.iso_curve(1, u0)), v, v);   // minor circle at u0
    int e_v = b.add_edge(b.add_curve_3d(srf.iso_curve(0, v0)), v, v);   // major circle at v0
    int si = b.add_surface(srf);
    b.add_pcurve(e_v, si, b.add_curve_2d(uv_line(u0, v0, u1, v0)), b.add_curve_2d(uv_line(u0, v1, u1, v1)));
    b.add_pcurve(e_u, si, b.add_curve_2d(uv_line(u1, v0, u1, v1)), b.add_curve_2d(uv_line(u0, v0, u0, v1)));
    int fi = b.add_face(si, {{b.add_wire({{e_v, F}, {e_u, F}, {e_v, R}, {e_u, R}}), F}});
    b.add_solid({{b.add_shell({{fi, F}}), F}});
    return b;
}

BRep BRep::create_block_with_hole(double sx, double sy, double sz, double hole_radius) {
    BRep b;
    b.name = "block_with_hole";
    double hz = sz * 0.5;
    box_corners(b, sx, sy, sz);
    PolyFaceBuilder pb{b, {}};
    std::vector<BRepRef> faces;
    for (int fi = 2; fi < 6; ++fi) {
        const int* fv = BOX_FACES[fi];
        faces.push_back({pb.face(quad_patch(b, fv), {fv[0], fv[1], fv[2], fv[3]}), F});
    }
    Point p_bot(hole_radius, 0, -hz), p_top(hole_radius, 0, hz);
    int v_bot = b.add_vertex(p_bot);
    int v_top = b.add_vertex(p_top);
    int e_bot = b.add_edge(b.add_curve_3d(Primitives::circle(0, 0, -hz, hole_radius)), v_bot, v_bot);
    int e_top = b.add_edge(b.add_curve_3d(Primitives::circle(0, 0, hz, hole_radius)), v_top, v_top);
    int e_seam = b.add_edge(b.add_curve_3d(NurbsCurve::create(false, 1, {p_bot, p_top})), v_bot, v_top);
    NurbsSurface bore = Primitives::cylinder_surface(0, 0, -hz, hole_radius, sz);
    faces.push_back({body_face(b, b.add_surface(bore), e_bot, e_seam, e_top), R});
    for (int fi = 0; fi < 2; ++fi) {
        const int* fv = BOX_FACES[fi];
        NurbsSurface cap = quad_patch(b, fv);
        std::vector<BRepRef> outer;
        int si = pb.face(cap, {fv[0], fv[1], fv[2], fv[3]}, &outer);
        int e_hole = fi == 0 ? e_bot : e_top;
        NurbsCurve c2d = project_to_patch(b.m_curves_3d[b.m_edges[e_hole].curve_3d_index], cap);
        BRepOrientation o = uv_signed_area(c2d) < 0.0 ? F : R;
        b.add_pcurve(e_hole, si, b.add_curve_2d(c2d));
        faces.push_back({b.add_face(si, {{b.add_wire(outer), F}, {b.add_wire({{e_hole, o}}), F}}), F});
    }
    b.add_solid({{b.add_shell(faces), F}});
    return b;
}

namespace {
/// Padded bilinear patch through `pts` in the plane (org, xa, ya); UV of each point returned.
NurbsSurface planar_patch_through(const std::vector<Point>& pts, const Point& org, const Vector& xa, const Vector& ya) {
    double umin = 1e30, umax = -1e30, vmin = 1e30, vmax = -1e30;
    for (const auto& p : pts) {
        double dx = p[0] - org[0], dy = p[1] - org[1], dz = p[2] - org[2];
        double u = dx*xa[0] + dy*xa[1] + dz*xa[2];
        double v = dx*ya[0] + dy*ya[1] + dz*ya[2];
        umin = std::min(umin, u); umax = std::max(umax, u);
        vmin = std::min(vmin, v); vmax = std::max(vmax, v);
    }
    double pad = std::max(umax - umin, vmax - vmin) * 0.01;
    umin -= pad; umax += pad; vmin -= pad; vmax += pad;
    auto pt3d = [&](double u, double v) {
        return Point(org[0] + u*xa[0] + v*ya[0], org[1] + u*xa[1] + v*ya[1], org[2] + u*xa[2] + v*ya[2]);
    };
    return bilinear_patch(pt3d(umin, vmin), pt3d(umax, vmin), pt3d(umin, vmax), pt3d(umax, vmax));
}

int find_or_add_vertex(BRep& b, const Point& p, double tol) {
    for (int i = 0; i < (int)b.m_vertices.size(); ++i)
        if (b.m_vertices[i].point.distance(p) < tol) return i;
    return b.add_vertex(p);
}

std::vector<Point> cv_points(const NurbsCurve& c) {
    std::vector<Point> pts;
    for (int k = 0; k < c.cv_count(); ++k) {
        auto [wx, wy, wz, w] = c.get_cv_4d(k);
        if (w != 0.0) pts.push_back(Point(wx/w, wy/w, wz/w));
    }
    return pts;
}

/// Signed volume of face meshes (positive when the windings point outward).
static double signed_volume(const std::vector<Mesh>& meshes) {
    double total = 0.0;
    for (const Mesh& fm : meshes)
        for (const auto& [fk, fverts] : fm.face)
            for (size_t k = 1; k + 1 < fverts.size(); ++k) {
                Point a = fm.vertex.at(fverts[0]).position();
                Point b = fm.vertex.at(fverts[k]).position();
                Point c = fm.vertex.at(fverts[k + 1]).position();
                total += a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0]) + a[2]*(b[0]*c[1]-b[1]*c[0]);
            }
    return total / 6.0;
}

/// BRepBuilderAPI_Sewing + MakeSolid for free faces: when every edge is shared by exactly two
/// face uses, orient the faces consistently across shared edges, one shell per connected
/// component wound outward, and one solid per shell.
static void close_free_faces(BRep& b) {
    int nf = b.face_count();
    if (nf == 0) return;
    std::vector<std::vector<std::pair<int, BRepOrientation>>> uses(b.m_edges.size());
    for (int fi = 0; fi < nf; ++fi)
        for (const auto& wr : b.m_faces[fi].wires)
            for (const auto& er : b.wire_edges(wr)) uses[er.index].push_back({fi, er.orientation});
    for (const auto& u : uses) if (u.size() != 2) return;
    std::vector<BRepOrientation> fo(nf, F);
    std::vector<bool> seen(nf, false);
    std::vector<std::vector<int>> components;
    for (int seed = 0; seed < nf; ++seed) {
        if (seen[seed]) continue;
        std::vector<int> comp, stack = {seed};
        seen[seed] = true;
        while (!stack.empty()) {
            int fi = stack.back(); stack.pop_back();
            comp.push_back(fi);
            for (const auto& wr : b.m_faces[fi].wires)
                for (const auto& er : b.wire_edges(wr))
                    for (const auto& [g, og] : uses[er.index]) {
                        if (g == fi || seen[g]) continue;
                        fo[g] = og == er.orientation ? brep_reverse(fo[fi]) : fo[fi];
                        seen[g] = true;
                        stack.push_back(g);
                    }
        }
        components.push_back(comp);
    }
    std::vector<BRepRef> shells;
    for (const auto& comp : components) {
        std::vector<BRepRef> refs;
        for (int fi : comp) refs.push_back({fi, fo[fi]});
        shells.push_back({b.add_shell(refs), F});
    }
    std::vector<Mesh> fm = b.face_meshes();
    for (const auto& sr : shells) {
        std::vector<Mesh> part;
        for (const auto& fr : b.m_shells[sr.index].faces) part.push_back(fm[fr.index]);
        if (signed_volume(part) < 0.0)
            for (auto& fr : b.m_shells[sr.index].faces) fr.orientation = brep_reverse(fr.orientation);
        b.add_solid({sr});
    }
}
} // namespace

BRep BRep::from_polylines(const std::vector<Polyline>& polylines) {
    BRep b;
    b.name = "polysurface";
    double tol = 1e-6;
    PolyFaceBuilder pb{b, {}};
    for (const auto& pl : polylines) {
        auto pts = pl.get_points();
        int n = pl.is_closed() ? (int)pts.size() - 1 : (int)pts.size();
        if (n < 3) continue;
        Point org; Plane plane;
        pl.get_fast_plane(org, plane);
        if (!plane.is_valid()) continue;
        std::vector<int> vi;
        for (int i = 0; i < n; ++i) vi.push_back(find_or_add_vertex(b, pts[i], tol));
        pts.resize(n);
        pb.face(planar_patch_through(pts, org, plane.x_axis(), plane.y_axis()), vi);
    }
    close_free_faces(b);
    return b;
}

BRep BRep::from_nurbscurves(const std::vector<NurbsCurve>& curves, const std::vector<std::vector<NurbsCurve>>& holes) {
    BRep b;
    b.name = "polysurface";
    double tol = 1e-6;
    auto curve_wire = [&](const NurbsCurve& crv, int si) {
        Point sp = crv.point_at(crv.domain().first);
        Point ep = crv.point_at(crv.domain().second);
        int vs = find_or_add_vertex(b, sp, tol);
        int ve = crv.is_closed() ? vs : find_or_add_vertex(b, ep, tol);
        int ei = b.add_edge(b.add_curve_3d(crv), vs, ve);
        b.add_pcurve(ei, si, b.add_curve_2d(project_to_patch(crv, b.m_surfaces[si])));
        return b.add_wire({{ei, F}});
    };
    for (int ci = 0; ci < (int)curves.size(); ++ci) {
        const auto& crv = curves[ci];
        auto pts = cv_points(crv);
        if (pts.size() >= 2 && pts.front().distance(pts.back()) < tol) pts.pop_back();
        if (pts.size() < 3) continue;
        Point org; Plane plane;
        Polyline(pts).get_fast_plane(org, plane);
        if (!plane.is_valid()) continue;
        if (ci < (int)holes.size())
            for (const auto& h : holes[ci]) { auto hp = cv_points(h); pts.insert(pts.end(), hp.begin(), hp.end()); }
        int si = b.add_surface(planar_patch_through(pts, org, plane.x_axis(), plane.y_axis()));
        std::vector<BRepRef> wires = {{curve_wire(crv, si), F}};
        if (ci < (int)holes.size())
            for (const auto& h : holes[ci]) wires.push_back({curve_wire(h, si), F});
        b.add_face(si, wires);
    }
    close_free_faces(b);
    return b;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors & Destructor
///////////////////////////////////////////////////////////////////////////////////////////

BRep::BRep() {}

BRep::BRep(const BRep& other) { *this = other; }

BRep& BRep::operator=(const BRep& other) {
    if (this == &other) return *this;
    _guid.clear();
    name = other.name;
    width = other.width;
    surfacecolor = other.surfacecolor;
    m_surfaces = other.m_surfaces;
    m_curves_3d = other.m_curves_3d;
    m_curves_2d = other.m_curves_2d;
    m_vertices = other.m_vertices;
    m_edges = other.m_edges;
    m_wires = other.m_wires;
    m_faces = other.m_faces;
    m_shells = other.m_shells;
    m_solids = other.m_solids;
    return *this;
}

bool BRep::operator==(const BRep& other) const {
    return name == other.name && width == other.width && surfacecolor == other.surfacecolor
        && m_surfaces.size() == other.m_surfaces.size()
        && m_vertices.size() == other.m_vertices.size()
        && m_edges.size() == other.m_edges.size()
        && m_wires.size() == other.m_wires.size()
        && m_faces.size() == other.m_faces.size()
        && m_shells.size() == other.m_shells.size()
        && m_solids.size() == other.m_solids.size();
}

bool BRep::operator!=(const BRep& other) const { return !(*this == other); }

BRep::~BRep() {}

///////////////////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////////////////

int BRep::vertex_count() const { return (int)m_vertices.size(); }
int BRep::edge_count() const { return (int)m_edges.size(); }
int BRep::wire_count() const { return (int)m_wires.size(); }
int BRep::face_count() const { return (int)m_faces.size(); }
int BRep::shell_count() const { return (int)m_shells.size(); }
int BRep::solid_count() const { return (int)m_solids.size(); }

bool BRep::is_valid() const {
    if (m_faces.empty()) return false;
    auto in = [](int i, size_t n) { return i >= 0 && i < (int)n; };
    for (const auto& e : m_edges) {
        if (!in(e.start_vertex, m_vertices.size()) || !in(e.end_vertex, m_vertices.size())) return false;
        if (!e.degenerated && !in(e.curve_3d_index, m_curves_3d.size())) return false;
        for (const auto& pc : e.pcurves) {
            if (!in(pc.surface_index, m_surfaces.size()) || !in(pc.curve_2d_index, m_curves_2d.size())) return false;
            if (pc.curve_2d_index_2 >= 0 && !in(pc.curve_2d_index_2, m_curves_2d.size())) return false;
        }
    }
    for (const auto& w : m_wires) {
        if (w.edges.empty()) return false;
        for (const auto& r : w.edges) if (!in(r.index, m_edges.size())) return false;
    }
    for (const auto& f : m_faces) {
        if (!in(f.surface_index, m_surfaces.size()) || f.wires.empty()) return false;
        for (const auto& r : f.wires) if (!in(r.index, m_wires.size())) return false;
    }
    for (const auto& s : m_shells)
        for (const auto& r : s.faces) if (!in(r.index, m_faces.size())) return false;
    for (const auto& s : m_solids)
        for (const auto& r : s.shells) if (!in(r.index, m_shells.size())) return false;
    return true;
}

bool BRep::is_closed(int shell_index) const {
    if (shell_index < 0 || shell_index >= (int)m_shells.size()) return false;
    std::vector<int> uses(m_edges.size(), 0);
    for (const auto& fr : m_shells[shell_index].faces)
        for (const auto& wr : m_faces[fr.index].wires)
            for (const auto& er : wire_edges(wr)) ++uses[er.index];
    for (size_t i = 0; i < m_edges.size(); ++i)
        if (!m_edges[i].degenerated && uses[i] != 0 && uses[i] != 2) return false;
    return !m_shells[shell_index].faces.empty();
}

bool BRep::is_solid() const {
    if (m_solids.empty()) return false;
    for (const auto& s : m_solids)
        for (const auto& r : s.shells)
            if (!is_closed(r.index)) return false;
    return true;
}

BRepOrientation BRep::face_orientation(int face_index) const {
    for (const auto& s : m_shells)
        for (const auto& r : s.faces)
            if (r.index == face_index) return r.orientation;
    return BRepOrientation::Forward;
}

int BRep::pcurve_index(int edge_index, int face_index, BRepOrientation orientation) const {
    if (edge_index < 0 || edge_index >= (int)m_edges.size()) return -1;
    if (face_index < 0 || face_index >= (int)m_faces.size()) return -1;
    int si = m_faces[face_index].surface_index;
    for (const auto& pc : m_edges[edge_index].pcurves)
        if (pc.surface_index == si)
            return (orientation == BRepOrientation::Reversed && pc.curve_2d_index_2 >= 0) ? pc.curve_2d_index_2 : pc.curve_2d_index;
    return -1;
}

std::vector<BRepRef> BRep::wire_edges(const BRepRef& wire) const {
    std::vector<BRepRef> out;
    if (wire.index < 0 || wire.index >= (int)m_wires.size()) return out;
    const auto& edges = m_wires[wire.index].edges;
    for (const auto& r : edges) out.push_back({r.index, brep_compose(wire.orientation, r.orientation)});
    if (wire.orientation == BRepOrientation::Reversed) std::reverse(out.begin(), out.end());
    return out;
}

std::vector<BRepRef> BRep::edge_faces(int edge_index) const {
    std::vector<BRepRef> out;
    for (int fi = 0; fi < (int)m_faces.size(); ++fi) {
        BRepOrientation fo = face_orientation(fi);
        for (const auto& wr : m_faces[fi].wires)
            for (const auto& er : wire_edges(wr))
                if (er.index == edge_index) out.push_back({fi, brep_compose(fo, er.orientation)});
    }
    return out;
}

std::vector<Point> BRep::vertex_points() const {
    std::vector<Point> pts;
    for (const auto& v : m_vertices) pts.push_back(v.point);
    return pts;
}

double BRep::update_tolerances() {
    double worst = 0.0;
    for (auto& e : m_edges) {
        BRepVertex& vs = m_vertices[e.start_vertex];
        BRepVertex& ve = m_vertices[e.end_vertex];
        double tol = e.tolerance;
        if (e.curve_3d_index >= 0) {
            const NurbsCurve& c = m_curves_3d[e.curve_3d_index];
            tol = std::max(tol, c.point_at(c.domain().first).distance(vs.point));
            tol = std::max(tol, c.point_at(c.domain().second).distance(ve.point));
        }
        for (const auto& pc : e.pcurves) {
            const NurbsSurface& srf = m_surfaces[pc.surface_index];
            for (int ci : {pc.curve_2d_index, pc.curve_2d_index_2}) {
                if (ci < 0) continue;
                const NurbsCurve& c2 = m_curves_2d[ci];
                Point a = c2.point_at(c2.domain().first), z = c2.point_at(c2.domain().second);
                tol = std::max(tol, srf.point_at(a[0], a[1]).distance(vs.point));
                tol = std::max(tol, srf.point_at(z[0], z[1]).distance(ve.point));
            }
        }
        e.tolerance = tol;
        vs.tolerance = std::max(vs.tolerance, tol);
        ve.tolerance = std::max(ve.tolerance, tol);
        worst = std::max(worst, tol);
    }
    return worst;
}

double BRep::volume() const { return mesh().volume(); }

///////////////////////////////////////////////////////////////////////////////////////////
// Building
///////////////////////////////////////////////////////////////////////////////////////////

int BRep::add_surface(const NurbsSurface& srf) { m_surfaces.push_back(srf); return (int)m_surfaces.size() - 1; }
int BRep::add_curve_3d(const NurbsCurve& crv) { m_curves_3d.push_back(crv); return (int)m_curves_3d.size() - 1; }
int BRep::add_curve_2d(const NurbsCurve& crv) { m_curves_2d.push_back(crv); return (int)m_curves_2d.size() - 1; }

int BRep::add_vertex(const Point& pt, double tolerance) {
    m_vertices.push_back({pt, tolerance});
    return (int)m_vertices.size() - 1;
}

int BRep::add_edge(int curve_3d_index, int start_vertex, int end_vertex, double tolerance) {
    BRepEdge e;
    e.curve_3d_index = curve_3d_index;
    e.start_vertex = start_vertex;
    e.end_vertex = end_vertex;
    e.tolerance = tolerance;
    e.degenerated = curve_3d_index < 0;
    m_edges.push_back(e);
    return (int)m_edges.size() - 1;
}

void BRep::add_pcurve(int edge_index, int surface_index, int curve_2d_index, int curve_2d_index_2) {
    for (auto& pc : m_edges[edge_index].pcurves)
        if (pc.surface_index == surface_index) {
            pc.curve_2d_index = curve_2d_index;
            pc.curve_2d_index_2 = curve_2d_index_2;
            return;
        }
    m_edges[edge_index].pcurves.push_back({surface_index, curve_2d_index, curve_2d_index_2});
}

int BRep::add_wire(const std::vector<BRepRef>& edges) {
    m_wires.push_back({edges});
    return (int)m_wires.size() - 1;
}

int BRep::add_face(int surface_index, const std::vector<BRepRef>& wires, double tolerance) {
    BRepFace f;
    f.surface_index = surface_index;
    f.wires = wires;
    f.tolerance = tolerance;
    m_faces.push_back(f);
    return (int)m_faces.size() - 1;
}

int BRep::add_shell(const std::vector<BRepRef>& faces) {
    m_shells.push_back({faces});
    return (int)m_shells.size() - 1;
}

int BRep::add_solid(const std::vector<BRepRef>& shells) {
    m_solids.push_back({shells});
    return (int)m_solids.size() - 1;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Meshing
///////////////////////////////////////////////////////////////////////////////////////////

std::vector<Point> BRep::wire_uv_points(int face_index, const BRepRef& wire) const {
    std::vector<Point> pts;
    for (const auto& er : wire_edges(wire)) {
        int ci = pcurve_index(er.index, face_index, er.orientation);
        if (ci < 0) continue;
        const NurbsCurve& crv = m_curves_2d[ci];
        std::vector<Point> seg;
        if (crv.degree() <= 1 && !crv.is_rational()) {
            for (int k = 0; k < crv.cv_count(); ++k) seg.push_back(crv.get_cv(k));
        } else {
            seg = crv.divide_by_count(std::max(crv.cv_count() * 4, 16), true).first;
        }
        if (er.orientation == BRepOrientation::Reversed) std::reverse(seg.begin(), seg.end());
        for (size_t k = 0; k + 1 < seg.size(); ++k) pts.push_back(seg[k]);
    }
    return pts;
}

Mesh BRep::mesh() const {
    std::vector<std::vector<Point>> polygons;
    for (const Mesh& fm : face_meshes()) {
        for (const auto& [fk, fverts] : fm.face) {
            std::vector<Point> poly;
            for (auto vi : fverts) poly.push_back(fm.vertex.at(vi).position());
            polygons.push_back(poly);
        }
    }
    return Mesh::from_polylines(polygons, 1e-6);
}

std::vector<Mesh> BRep::face_meshes() const {
    return face_meshes_q(false, 0.0, 0.0);
}

std::vector<Mesh> BRep::face_meshes_q(bool has_quality, double max_angle_deg, double chord_factor) const {
    int nf = (int)m_faces.size();

    // Phase 1: a face whose outer wire is the full UV rectangle (straight pcurves enclosing the
    // whole domain area, no holes) is meshed directly on the surface grid; everything else goes
    // through the trimmed CDT.
    std::vector<bool> face_direct(nf, false);
    for (int fi = 0; fi < nf; ++fi) {
        const auto& face = m_faces[fi];
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        if (face.wires.size() != 1) continue;
        bool all_linear = true;
        for (const auto& er : wire_edges(face.wires[0])) {
            int ci = pcurve_index(er.index, fi, er.orientation);
            if (ci < 0) continue;
            if (m_curves_2d[ci].degree() > 1 || m_curves_2d[ci].is_rational()) all_linear = false;
        }
        if (!all_linear) continue;
        std::vector<Point> outer = wire_uv_points(fi, face.wires[0]);
        if (outer.size() < 3) continue;
        auto [u0, u1] = srf.domain(0);
        auto [v0, v1] = srf.domain(1);
        double area = 0.0;
        for (size_t i = 0; i < outer.size(); ++i) {
            const Point& a = outer[i];
            const Point& c = outer[(i + 1) % outer.size()];
            area += a[0] * c[1] - c[0] * a[1];
        }
        double domain_area = (u1 - u0) * (v1 - v0);
        face_direct[fi] = std::abs(std::abs(area) * 0.5 - domain_area) < 1e-3 * domain_area;
    }

    // Phase 2: direct faces. Record the 3D boundary discretisation along every edge shared
    // with a CDT face so both sides tessellate the seam with the same points.
    std::vector<Mesh> fmesh(nf);
    std::map<int, std::vector<Point>> edge_bnd;
    for (int fi = 0; fi < nf; ++fi) {
        if (!face_direct[fi]) continue;
        const auto& face = m_faces[fi];
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        fmesh[fi] = has_quality
            ? RemeshNurbsSurfaceGrid::from_u_v_q(srf, 0, 0, max_angle_deg, chord_factor)
            : srf.mesh();
        auto [u0, u1] = srf.domain(0);
        auto [v0, v1] = srf.domain(1);
        double utol = (u1 - u0) * 0.001, vtol = (v1 - v0) * 0.001;
        for (const auto& er : wire_edges(face.wires[0])) {
            int eidx = er.index;
            if (edge_bnd.count(eidx)) continue;
            bool shared = false;
            for (const auto& fr : edge_faces(eidx))
                if (fr.index != fi && !face_direct[fr.index]) { shared = true; break; }
            if (!shared) continue;
            int ci = pcurve_index(eidx, fi, er.orientation);
            if (ci < 0) continue;
            const NurbsCurve& c2d = m_curves_2d[ci];
            Point sp = c2d.get_cv(0), ep = c2d.get_cv(c2d.cv_count() - 1);
            bool at_v0 = std::abs(sp[1] - v0) < vtol && std::abs(ep[1] - v0) < vtol;
            bool at_v1 = std::abs(sp[1] - v1) < vtol && std::abs(ep[1] - v1) < vtol;
            bool at_u0 = std::abs(sp[0] - u0) < utol && std::abs(ep[0] - u0) < utol;
            bool at_u1 = std::abs(sp[0] - u1) < utol && std::abs(ep[0] - u1) < utol;
            if (!at_v0 && !at_v1 && !at_u0 && !at_u1) continue;
            std::vector<std::pair<double, Point>> pts;
            for (auto& [vk, vd] : fmesh[fi].vertex) {
                auto iu = vd.attributes.find("u"), iv = vd.attributes.find("v");
                if (iu == vd.attributes.end() || iv == vd.attributes.end()) continue;
                if (at_v0 && std::abs(iv->second - v0) < vtol * 0.1) pts.push_back({iu->second, vd.position()});
                else if (at_v1 && std::abs(iv->second - v1) < vtol * 0.1) pts.push_back({iu->second, vd.position()});
                else if (at_u0 && std::abs(iu->second - u0) < utol * 0.1) pts.push_back({iv->second, vd.position()});
                else if (at_u1 && std::abs(iu->second - u1) < utol * 0.1) pts.push_back({iv->second, vd.position()});
            }
            std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
            if (pts.size() >= 2) {
                std::vector<Point> bnd;
                for (auto& [p, pt] : pts) bnd.push_back(pt);
                edge_bnd[eidx] = bnd;
            }
        }
    }

    // Phase 3: CDT faces. Shared edges reuse the direct face's boundary points projected into
    // this face's planar patch; every other edge samples its own pcurve.
    for (int fi = 0; fi < nf; ++fi) {
        if (face_direct[fi]) continue;
        const auto& face = m_faces[fi];
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        Point p00 = srf.get_cv(0, 0), p10 = srf.get_cv(1, 0), p01 = srf.get_cv(0, 1);
        double eu[3] = {p10[0]-p00[0], p10[1]-p00[1], p10[2]-p00[2]};
        double ev[3] = {p01[0]-p00[0], p01[1]-p00[1], p01[2]-p00[2]};
        double eu2 = eu[0]*eu[0]+eu[1]*eu[1]+eu[2]*eu[2], ev2 = ev[0]*ev[0]+ev[1]*ev[1]+ev[2]*ev[2];
        bool can_project = srf.degree(0) == 1 && srf.degree(1) == 1 && eu2 > 1e-28 && ev2 > 1e-28;

        NurbsSurfaceTrimmed ts;
        ts.m_surface = srf;
        for (size_t wi = 0; wi < face.wires.size(); ++wi) {
            std::vector<Point> loop_pts;
            for (const auto& er : wire_edges(face.wires[wi])) {
                int ci = pcurve_index(er.index, fi, er.orientation);
                if (ci < 0) continue;
                const NurbsCurve& crv = m_curves_2d[ci];
                std::vector<Point> seg;
                if (can_project && edge_bnd.count(er.index)) {
                    for (const auto& pt : edge_bnd[er.index]) {
                        double dx = pt[0]-p00[0], dy = pt[1]-p00[1], dz = pt[2]-p00[2];
                        seg.push_back(Point((dx*eu[0]+dy*eu[1]+dz*eu[2]) / eu2, (dx*ev[0]+dy*ev[1]+dz*ev[2]) / ev2, 0));
                    }
                    Point start = crv.point_at(er.orientation == BRepOrientation::Reversed ? crv.domain().second : crv.domain().first);
                    if (seg.front().distance(start) > seg.back().distance(start)) std::reverse(seg.begin(), seg.end());
                } else {
                    if (crv.degree() <= 1 && !crv.is_rational()) {
                        for (int k = 0; k < crv.cv_count(); ++k) seg.push_back(crv.get_cv(k));
                    } else {
                        seg = crv.divide_by_count(std::max(crv.cv_count() * 4, 16), true).first;
                    }
                    if (er.orientation == BRepOrientation::Reversed) std::reverse(seg.begin(), seg.end());
                }
                for (size_t k = 0; k + 1 < seg.size(); ++k) loop_pts.push_back(seg[k]);
            }
            if (loop_pts.size() < 3) continue;
            NurbsCurve loop_crv = NurbsCurve::create(true, 1, loop_pts);
            if (wi == 0) ts.m_outer_loop = loop_crv;
            else ts.m_inner_loops.push_back(loop_crv);
        }
        fmesh[fi] = ts.mesh();
    }

    // A Reversed face has its outward normal opposite to the surface normal: flip winding
    // and stored normals together so shading agrees with the geometry.
    for (int fi = 0; fi < nf; ++fi) {
        if (face_orientation(fi) != BRepOrientation::Reversed) continue;
        fmesh[fi].flip();
        for (auto& [vk, vd] : fmesh[fi].vertex) {
            auto n = vd.normal();
            if (n) vd.set_normal(-(*n)[0], -(*n)[1], -(*n)[2]);
        }
    }
    return fmesh;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Evaluation
///////////////////////////////////////////////////////////////////////////////////////////

Point BRep::point_at(int face_index, double u, double v) const {
    if (face_index < 0 || face_index >= (int)m_faces.size()) return Point();
    return m_surfaces[m_faces[face_index].surface_index].point_at(u, v);
}

Vector BRep::normal_at(int face_index, double u, double v) const {
    if (face_index < 0 || face_index >= (int)m_faces.size()) return Vector();
    Vector n = m_surfaces[m_faces[face_index].surface_index].normal_at(u, v);
    if (face_orientation(face_index) == BRepOrientation::Reversed) return Vector(-n[0], -n[1], -n[2]);
    return n;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void BRep::transform(const Xform& xform) {
    for (auto& srf : m_surfaces) srf.transform(xform);
    for (auto& crv : m_curves_3d) crv.transform(xform);
    for (auto& v : m_vertices) v.point = xform.transform_point(v.point);
}

BRep BRep::transformed(const Xform& xform) const {
    BRep b = *this;
    b.transform(xform);
    return b;
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

static nlohmann::ordered_json refs_to_json(const std::vector<BRepRef>& refs) {
    nlohmann::ordered_json arr = nlohmann::ordered_json::array();
    for (const auto& r : refs) {
        nlohmann::ordered_json rj;
        rj["index"] = r.index;
        rj["orientation"] = orientation_to_str(r.orientation);
        arr.push_back(rj);
    }
    return arr;
}

static std::vector<BRepRef> refs_from_json(const nlohmann::json& arr) {
    std::vector<BRepRef> refs;
    for (const auto& r : arr) refs.push_back({r["index"], orientation_from_str(r["orientation"])});
    return refs;
}

nlohmann::ordered_json BRep::jsondump() const {
    nlohmann::ordered_json j;
    j["curves_2d"] = nlohmann::ordered_json::array();
    for (const auto& c : m_curves_2d) j["curves_2d"].push_back(c.jsondump());
    j["curves_3d"] = nlohmann::ordered_json::array();
    for (const auto& c : m_curves_3d) j["curves_3d"].push_back(c.jsondump());
    j["edges"] = nlohmann::ordered_json::array();
    for (const auto& e : m_edges) {
        nlohmann::ordered_json ej;
        ej["curve_3d_index"] = e.curve_3d_index;
        ej["degenerated"] = e.degenerated;
        ej["end_vertex"] = e.end_vertex;
        ej["pcurves"] = nlohmann::ordered_json::array();
        for (const auto& pc : e.pcurves) {
            nlohmann::ordered_json pj;
            pj["curve_2d_index"] = pc.curve_2d_index;
            pj["curve_2d_index_2"] = pc.curve_2d_index_2;
            pj["surface_index"] = pc.surface_index;
            ej["pcurves"].push_back(pj);
        }
        ej["start_vertex"] = e.start_vertex;
        ej["tolerance"] = e.tolerance;
        j["edges"].push_back(ej);
    }
    j["faces"] = nlohmann::ordered_json::array();
    for (const auto& f : m_faces) {
        nlohmann::ordered_json fj;
        if (f.facecolor.a > 0) fj["facecolor"] = f.facecolor.jsondump();
        fj["surface_index"] = f.surface_index;
        fj["tolerance"] = f.tolerance;
        fj["wires"] = refs_to_json(f.wires);
        j["faces"].push_back(fj);
    }
    j["guid"] = guid();
    j["name"] = name;
    j["shells"] = nlohmann::ordered_json::array();
    for (const auto& s : m_shells) {
        nlohmann::ordered_json sj;
        sj["faces"] = refs_to_json(s.faces);
        j["shells"].push_back(sj);
    }
    j["solids"] = nlohmann::ordered_json::array();
    for (const auto& s : m_solids) {
        nlohmann::ordered_json sj;
        sj["shells"] = refs_to_json(s.shells);
        j["solids"].push_back(sj);
    }
    j["surfacecolor"] = surfacecolor.jsondump();
    j["surfaces"] = nlohmann::ordered_json::array();
    for (const auto& s : m_surfaces) j["surfaces"].push_back(s.jsondump());
    j["type"] = "BRep";
    j["vertices"] = nlohmann::ordered_json::array();
    for (const auto& v : m_vertices) {
        nlohmann::ordered_json vj;
        vj["point"] = nlohmann::ordered_json::array({v.point[0], v.point[1], v.point[2]});
        vj["tolerance"] = v.tolerance;
        j["vertices"].push_back(vj);
    }
    j["width"] = width;
    j["wires"] = nlohmann::ordered_json::array();
    for (const auto& w : m_wires) {
        nlohmann::ordered_json wj;
        wj["edges"] = refs_to_json(w.edges);
        j["wires"].push_back(wj);
    }
    return j;
}

BRep BRep::jsonload(const nlohmann::json& data) {
    BRep b;
    if (data.contains("guid")) b.guid() = data["guid"];
    if (data.contains("name")) b.name = data["name"];
    if (data.contains("width")) b.width = data["width"];
    if (data.contains("surfacecolor")) b.surfacecolor = Color::jsonload(data["surfacecolor"]);
    if (data.contains("curves_2d"))
        for (const auto& c : data["curves_2d"]) b.m_curves_2d.push_back(NurbsCurve::jsonload(c));
    if (data.contains("curves_3d"))
        for (const auto& c : data["curves_3d"]) b.m_curves_3d.push_back(NurbsCurve::jsonload(c));
    if (data.contains("surfaces"))
        for (const auto& s : data["surfaces"]) b.m_surfaces.push_back(NurbsSurface::jsonload(s));
    if (data.contains("vertices"))
        for (const auto& v : data["vertices"])
            b.m_vertices.push_back({Point(v["point"][0], v["point"][1], v["point"][2]), v["tolerance"]});
    if (data.contains("edges"))
        for (const auto& e : data["edges"]) {
            BRepEdge be;
            be.curve_3d_index = e["curve_3d_index"];
            be.degenerated = e["degenerated"];
            be.end_vertex = e["end_vertex"];
            for (const auto& pc : e["pcurves"])
                be.pcurves.push_back({pc["surface_index"], pc["curve_2d_index"], pc["curve_2d_index_2"]});
            be.start_vertex = e["start_vertex"];
            be.tolerance = e["tolerance"];
            b.m_edges.push_back(be);
        }
    if (data.contains("wires"))
        for (const auto& w : data["wires"]) b.m_wires.push_back({refs_from_json(w["edges"])});
    if (data.contains("faces"))
        for (const auto& f : data["faces"]) {
            BRepFace bf;
            if (f.contains("facecolor")) bf.facecolor = Color::jsonload(f["facecolor"]);
            bf.surface_index = f["surface_index"];
            bf.tolerance = f["tolerance"];
            bf.wires = refs_from_json(f["wires"]);
            b.m_faces.push_back(bf);
        }
    if (data.contains("shells"))
        for (const auto& s : data["shells"]) b.m_shells.push_back({refs_from_json(s["faces"])});
    if (data.contains("solids"))
        for (const auto& s : data["solids"]) b.m_solids.push_back({refs_from_json(s["shells"])});
    return b;
}

std::string BRep::file_json_dumps() const { return jsondump().dump(); }

BRep BRep::file_json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void BRep::file_json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

BRep BRep::file_json_load(const std::string& filename) {
    std::ifstream file(filename);
    nlohmann::json data;
    file >> data;
    return jsonload(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// Protobuf Serialization
///////////////////////////////////////////////////////////////////////////////////////////

static void refs_to_proto(const std::vector<BRepRef>& refs, google::protobuf::RepeatedPtrField<session_proto::BRepRef>* out) {
    for (const auto& r : refs) {
        auto* p = out->Add();
        p->set_index(r.index);
        p->set_orientation(static_cast<session_proto::BRepOrientation>(r.orientation));
    }
}

static std::vector<BRepRef> refs_from_proto(const google::protobuf::RepeatedPtrField<session_proto::BRepRef>& in) {
    std::vector<BRepRef> refs;
    for (const auto& r : in) refs.push_back({r.index(), static_cast<BRepOrientation>(r.orientation())});
    return refs;
}

std::string BRep::pb_dumps() const {
    session_proto::BRep proto;
    proto.set_guid(guid());
    proto.set_name(name);
    proto.set_width(width);
    for (const auto& c : m_curves_2d) proto.add_curves_2d()->ParseFromString(c.pb_dumps());
    for (const auto& c : m_curves_3d) proto.add_curves_3d()->ParseFromString(c.pb_dumps());
    for (const auto& s : m_surfaces) proto.add_surfaces()->ParseFromString(s.pb_dumps());
    for (const auto& v : m_vertices) {
        auto* p = proto.add_vertices();
        p->mutable_point()->set_x(v.point[0]);
        p->mutable_point()->set_y(v.point[1]);
        p->mutable_point()->set_z(v.point[2]);
        p->set_tolerance(v.tolerance);
    }
    for (const auto& e : m_edges) {
        auto* p = proto.add_edges();
        p->set_curve_3d_index(e.curve_3d_index);
        p->set_start_vertex(e.start_vertex);
        p->set_end_vertex(e.end_vertex);
        p->set_tolerance(e.tolerance);
        p->set_degenerated(e.degenerated);
        for (const auto& pc : e.pcurves) {
            auto* q = p->add_pcurves();
            q->set_surface_index(pc.surface_index);
            q->set_curve_2d_index(pc.curve_2d_index);
            q->set_curve_2d_index_2(pc.curve_2d_index_2);
        }
    }
    for (const auto& w : m_wires) refs_to_proto(w.edges, proto.add_wires()->mutable_edges());
    for (const auto& f : m_faces) {
        auto* p = proto.add_faces();
        p->set_surface_index(f.surface_index);
        refs_to_proto(f.wires, p->mutable_wires());
        p->set_tolerance(f.tolerance);
        if (f.facecolor.a > 0) {
            auto* fc = p->mutable_facecolor();
            fc->set_r(f.facecolor.r); fc->set_g(f.facecolor.g);
            fc->set_b(f.facecolor.b); fc->set_a(f.facecolor.a);
        }
    }
    for (const auto& s : m_shells) refs_to_proto(s.faces, proto.add_shells()->mutable_faces());
    for (const auto& s : m_solids) refs_to_proto(s.shells, proto.add_solids()->mutable_shells());
    auto* color_proto = proto.mutable_surfacecolor();
    color_proto->set_name(surfacecolor.name);
    color_proto->set_r(surfacecolor.r);
    color_proto->set_g(surfacecolor.g);
    color_proto->set_b(surfacecolor.b);
    color_proto->set_a(surfacecolor.a);
    return proto.SerializeAsString();
}

BRep BRep::pb_loads(const std::string& data) {
    session_proto::BRep proto;
    proto.ParseFromString(data);
    BRep b;
    b.guid() = proto.guid();
    b.name = proto.name();
    b.width = proto.width();
    for (const auto& c : proto.curves_2d()) b.m_curves_2d.push_back(NurbsCurve::pb_loads(c.SerializeAsString()));
    for (const auto& c : proto.curves_3d()) b.m_curves_3d.push_back(NurbsCurve::pb_loads(c.SerializeAsString()));
    for (const auto& s : proto.surfaces()) b.m_surfaces.push_back(NurbsSurface::pb_loads(s.SerializeAsString()));
    for (const auto& v : proto.vertices())
        b.m_vertices.push_back({Point(v.point().x(), v.point().y(), v.point().z()), v.tolerance()});
    for (const auto& e : proto.edges()) {
        BRepEdge be;
        be.curve_3d_index = e.curve_3d_index();
        be.start_vertex = e.start_vertex();
        be.end_vertex = e.end_vertex();
        be.tolerance = e.tolerance();
        be.degenerated = e.degenerated();
        for (const auto& pc : e.pcurves())
            be.pcurves.push_back({pc.surface_index(), pc.curve_2d_index(), pc.curve_2d_index_2()});
        b.m_edges.push_back(be);
    }
    for (const auto& w : proto.wires()) b.m_wires.push_back({refs_from_proto(w.edges())});
    for (const auto& f : proto.faces()) {
        BRepFace bf;
        bf.surface_index = f.surface_index();
        bf.wires = refs_from_proto(f.wires());
        bf.tolerance = f.tolerance();
        if (f.has_facecolor())
            bf.facecolor = Color(f.facecolor().r(), f.facecolor().g(), f.facecolor().b(), f.facecolor().a());
        b.m_faces.push_back(bf);
    }
    for (const auto& s : proto.shells()) b.m_shells.push_back({refs_from_proto(s.faces())});
    for (const auto& s : proto.solids()) b.m_solids.push_back({refs_from_proto(s.shells())});
    const auto& cp = proto.surfacecolor();
    b.surfacecolor.name = cp.name();
    b.surfacecolor.r = cp.r();
    b.surfacecolor.g = cp.g();
    b.surfacecolor.b = cp.b();
    b.surfacecolor.a = cp.a();
    return b;
}

void BRep::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

BRep BRep::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return pb_loads(data);
}

///////////////////////////////////////////////////////////////////////////////////////////
// String Representation
///////////////////////////////////////////////////////////////////////////////////////////

std::string BRep::str() const {
    return fmt::format("BRep(name={}, faces={}, edges={}, vertices={})",
                       name, face_count(), edge_count(), vertex_count());
}

std::string BRep::repr() const {
    return fmt::format("BRep(\n  name={},\n  faces={},\n  edges={},\n  vertices={},\n  solid={}\n)",
                       name, face_count(), edge_count(), vertex_count(),
                       is_solid() ? "true" : "false");
}

std::ostream& operator<<(std::ostream& os, const BRep& brep) {
    os << brep.str();
    return os;
}

} // namespace session_cpp
