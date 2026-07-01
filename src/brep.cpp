#include "brep.h"
#include "remesh_nurbssurface_grid.h"
#include "remesh_cdt.h"
#include "primitives.h"
#include "intersection.h"
#include "closest.h"
#include "plane.h"
#include "line.h"
#include "polyline.h"
#include "fmt/core.h"
#include <fstream>
#include <cmath>
#include <map>
#include <array>
#include <functional>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include "brep.pb.h"

namespace session_cpp {

///////////////////////////////////////////////////////////////////////////////////////////
// Constructors
///////////////////////////////////////////////////////////////////////////////////////////

BRep::BRep() {}

BRep::BRep(const BRep& other) { deep_copy_from(other); }

BRep& BRep::operator=(const BRep& other) {
    if (this != &other) deep_copy_from(other);
    return *this;
}

bool BRep::operator==(const BRep& other) const {
    if (name != other.name) return false;
    if (width != other.width) return false;
    if (surfacecolor != other.surfacecolor) return false;
    if (xform != other.xform) return false;
    if (m_faces.size() != other.m_faces.size()) return false;
    if (m_surfaces.size() != other.m_surfaces.size()) return false;
    if (m_topology_edges.size() != other.m_topology_edges.size()) return false;
    if (m_vertices.size() != other.m_vertices.size()) return false;
    return true;
}

bool BRep::operator!=(const BRep& other) const { return !(*this == other); }

BRep::~BRep() {}

void BRep::deep_copy_from(const BRep& src) {
    _guid.clear();
    name = src.name;
    width = src.width;
    surfacecolor = src.surfacecolor;
    xform = src.xform;
    m_surfaces = src.m_surfaces;
    m_curves_3d = src.m_curves_3d;
    m_curves_2d = src.m_curves_2d;
    m_vertices = src.m_vertices;
    m_topology_vertices = src.m_topology_vertices;
    m_topology_edges = src.m_topology_edges;
    m_trims = src.m_trims;
    m_loops = src.m_loops;
    m_faces = src.m_faces;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Factory
///////////////////////////////////////////////////////////////////////////////////////////

BRep BRep::create_box(double sx, double sy, double sz) {
    BRep brep;
    brep.name = "box";
    double hx = sx * 0.5, hy = sy * 0.5, hz = sz * 0.5;

    // 8 vertices: corners of the box
    Point corners[8] = {
        Point(-hx, -hy, -hz),
        Point( hx, -hy, -hz),
        Point( hx,  hy, -hz),
        Point(-hx,  hy, -hz),
        Point(-hx, -hy,  hz),
        Point( hx, -hy,  hz),
        Point( hx,  hy,  hz),
        Point(-hx,  hy,  hz),
    };
    for (int i = 0; i < 8; ++i) brep.add_vertex(corners[i]);

    // 6 faces: bottom(0), top(1), front(2), right(3), back(4), left(5)
    // Each face defined by 4 corner indices (CCW from outside)
    int face_verts[6][4] = {
        {0, 3, 2, 1}, // bottom (z=-hz), normal -Z
        {4, 5, 6, 7}, // top (z=+hz), normal +Z
        {0, 1, 5, 4}, // front (y=-hy), normal -Y
        {1, 2, 6, 5}, // right (x=+hx), normal +X
        {2, 3, 7, 6}, // back (y=+hy), normal +Y
        {3, 0, 4, 7}, // left (x=-hx), normal -X
    };

    // 12 edges: each edge defined by (start_vertex, end_vertex)
    int edge_verts[12][2] = {
        {0,1},{1,2},{2,3},{3,0},  // bottom edges
        {4,5},{5,6},{6,7},{7,4},  // top edges
        {0,4},{1,5},{2,6},{3,7},  // vertical edges
    };

    // Create 3D edge curves (lines)
    for (int i = 0; i < 12; ++i) {
        Point p0 = corners[edge_verts[i][0]];
        Point p1 = corners[edge_verts[i][1]];
        NurbsCurve line = NurbsCurve::create(false, 1, {p0, p1});
        brep.add_curve_3d(line);
    }

    // Create topology vertices
    for (int i = 0; i < 8; ++i) {
        BRepVertex tv;
        tv.point_index = i;
        brep.m_topology_vertices.push_back(tv);
    }

    // Create topology edges
    for (int i = 0; i < 12; ++i)
        brep.add_edge(i, edge_verts[i][0], edge_verts[i][1]);

    // For each face: create bilinear surface, outer loop with 4 trims
    for (int fi = 0; fi < 6; ++fi) {
        int* fv = face_verts[fi];
        Point p00 = corners[fv[0]], p10 = corners[fv[1]];
        Point p01 = corners[fv[3]], p11 = corners[fv[2]];

        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, p00); srf.set_cv(1, 0, p10);
        srf.set_cv(0, 1, p01); srf.set_cv(1, 1, p11);
        int si = brep.add_surface(srf);

        int face_idx = brep.add_face(si, false);
        int loop_idx = brep.add_loop(face_idx, BRepLoopType::Outer);

        // 4 trim curves in UV space (unit square boundary)
        Point uv_corners[4] = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };
        // Edge mapping for each face: which of the 12 edges correspond to each face edge
        // We find edges by matching vertex pairs
        auto find_edge = [&](int v0, int v1) -> int {
            for (int e = 0; e < 12; ++e) {
                if ((edge_verts[e][0] == v0 && edge_verts[e][1] == v1) ||
                    (edge_verts[e][0] == v1 && edge_verts[e][1] == v0))
                    return e;
            }
            return -1;
        };

        for (int ei = 0; ei < 4; ++ei) {
            int next = (ei + 1) % 4;
            NurbsCurve trim_crv = NurbsCurve::create(false, 1,
                {uv_corners[ei], uv_corners[next]});
            int c2d_idx = brep.add_curve_2d(trim_crv);
            int edge_idx = find_edge(fv[ei], fv[next]);
            bool rev = (edge_verts[edge_idx][0] != fv[ei]);
            brep.add_trim(c2d_idx, edge_idx, loop_idx, rev, BRepTrimType::Mated);
        }
    }

    // Update topology vertex edge indices
    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        auto& e = brep.m_topology_edges[ei];
        brep.m_topology_vertices[e.start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[e.end_vertex].edge_indices.push_back(ei);
    }

    return brep;
}

BRep BRep::create_cylinder(double radius, double height) {
    BRep brep;
    brep.name = "cylinder";
    NurbsSurface body = Primitives::cylinder_surface(0, 0, 0, radius, height);
    auto dom_u = body.domain(0);
    auto dom_v = body.domain(1);

    Point p_bot = body.point_at_corner(0, 0);
    Point p_top = body.point_at_corner(0, 1);
    int vi_bot = brep.add_vertex(p_bot);
    int vi_top = brep.add_vertex(p_top);
    brep.m_topology_vertices.push_back({vi_bot, {}});
    brep.m_topology_vertices.push_back({vi_top, {}});

    NurbsCurve circle_bot = Primitives::circle(0, 0, 0, radius);
    NurbsCurve circle_top = Primitives::circle(0, 0, height, radius);
    NurbsCurve seam_line = NurbsCurve::create(false, 1, {p_bot, p_top});
    int ci_bot = brep.add_curve_3d(circle_bot);
    int ci_top = brep.add_curve_3d(circle_top);
    int ci_seam = brep.add_curve_3d(seam_line);
    int ei_bot = brep.add_edge(ci_bot, 0, 0);
    int ei_top = brep.add_edge(ci_top, 1, 1);
    int ei_seam = brep.add_edge(ci_seam, 0, 1);

    int si_body = brep.add_surface(body);
    NurbsSurface cap_bot; cap_bot.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
    cap_bot.set_cv(0, 0, Point(-radius, -radius, 0));
    cap_bot.set_cv(1, 0, Point(radius, -radius, 0));
    cap_bot.set_cv(0, 1, Point(-radius, radius, 0));
    cap_bot.set_cv(1, 1, Point(radius, radius, 0));
    int si_bot = brep.add_surface(cap_bot);
    NurbsSurface cap_top; cap_top.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
    cap_top.set_cv(0, 0, Point(-radius, -radius, height));
    cap_top.set_cv(1, 0, Point(radius, -radius, height));
    cap_top.set_cv(0, 1, Point(-radius, radius, height));
    cap_top.set_cv(1, 1, Point(radius, radius, height));
    int si_top = brep.add_surface(cap_top);

    int fi_body = brep.add_face(si_body, false);
    int li_body = brep.add_loop(fi_body, BRepLoopType::Outer);
    auto c2d_bot = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.first, 0),
        Point(dom_u.second, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_bot), ei_bot, li_body, false, BRepTrimType::Mated);
    auto c2d_sr = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.first, 0),
        Point(dom_u.second, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li_body, false, BRepTrimType::Seam);
    auto c2d_top = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.second, 0),
        Point(dom_u.first, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_top), ei_top, li_body, true, BRepTrimType::Mated);
    auto c2d_sl = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.second, 0),
        Point(dom_u.first, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sl), ei_seam, li_body, true, BRepTrimType::Seam);

    // Circular 2D trim in UV space: circle at (0.5,0.5) radius 0.5
    const double cw = std::sqrt(2.0) / 2.0;
    double ccx[] = {1,1,0,-1,-1,-1,0,1,1}, ccy[] = {0,1,1,1,0,-1,-1,-1,0};
    double cwt[] = {1,cw,1,cw,1,cw,1,cw,1}, ckn[] = {0,0,1,1,2,2,3,3,4,4};
    auto make_cap_circle = [&]() {
        NurbsCurve c(3, true, 3, 9);
        for (int i = 0; i < 10; i++) c.set_nurbsknot(i, ckn[i]);
        for (int i = 0; i < 9; i++)
            c.set_cv_4d(i, (0.5+0.5*ccx[i])*cwt[i], (0.5+0.5*ccy[i])*cwt[i], 0.0, cwt[i]);
        return c;
    };

    int fi_bot = brep.add_face(si_bot, true);
    int li_bot = brep.add_loop(fi_bot, BRepLoopType::Outer);
    brep.add_trim(brep.add_curve_2d(make_cap_circle()), ei_bot, li_bot, true, BRepTrimType::Mated);

    int fi_top = brep.add_face(si_top, false);
    int li_top = brep.add_loop(fi_top, BRepLoopType::Outer);
    brep.add_trim(brep.add_curve_2d(make_cap_circle()), ei_top, li_top, false, BRepTrimType::Mated);

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::create_sphere(double radius) {
    BRep brep;
    brep.name = "sphere";
    NurbsSurface srf = Primitives::sphere_surface(0, 0, 0, radius);
    auto dom_u = srf.domain(0);
    auto dom_v = srf.domain(1);

    Point p_south(0, 0, -radius);
    Point p_north(0, 0, radius);
    int vi_south = brep.add_vertex(p_south);
    int vi_north = brep.add_vertex(p_north);
    brep.m_topology_vertices.push_back({vi_south, {}});
    brep.m_topology_vertices.push_back({vi_north, {}});

    NurbsCurve seam_crv = NurbsCurve::create(false, 1, {p_south, p_north});
    int ci_seam = brep.add_curve_3d(seam_crv);
    int ei_seam = brep.add_edge(ci_seam, 0, 1);

    int si = brep.add_surface(srf);
    int fi = brep.add_face(si, false);
    int li = brep.add_loop(fi, BRepLoopType::Outer);

    auto c2d_south = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.first, 0),
        Point(dom_u.second, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_south), -1, li, false, BRepTrimType::Singular);
    auto c2d_sr = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.first, 0),
        Point(dom_u.second, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li, false, BRepTrimType::Seam);
    auto c2d_north = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.second, 0),
        Point(dom_u.first, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_north), -1, li, false, BRepTrimType::Singular);
    auto c2d_sl = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.second, 0),
        Point(dom_u.first, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sl), ei_seam, li, true, BRepTrimType::Seam);

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::create_cone(double radius, double height) {
    // Side face = cone_surface (u in [0,4] = circle, v in [0,1] = base->apex; v=1 is a SINGULAR
    // apex pole, like a sphere pole) + one planar base cap. Mirrors create_cylinder's base+seam but
    // with a singular apex instead of a top cap.
    BRep brep;
    brep.name = "cone";
    NurbsSurface body = Primitives::cone_surface(0, 0, 0, radius, height);
    auto dom_u = body.domain(0);
    auto dom_v = body.domain(1);

    Point p_base = body.point_at_corner(0, 0);   // on base circle at u=0
    Point p_apex(0, 0, height);
    int vi_base = brep.add_vertex(p_base);
    int vi_apex = brep.add_vertex(p_apex);
    brep.m_topology_vertices.push_back({vi_base, {}});
    brep.m_topology_vertices.push_back({vi_apex, {}});

    NurbsCurve circle_base = Primitives::circle(0, 0, 0, radius);
    NurbsCurve seam_line = NurbsCurve::create(false, 1, {p_base, p_apex});
    int ci_base = brep.add_curve_3d(circle_base);
    int ci_seam = brep.add_curve_3d(seam_line);
    int ei_base = brep.add_edge(ci_base, vi_base, vi_base);
    int ei_seam = brep.add_edge(ci_seam, vi_base, vi_apex);

    int si_body = brep.add_surface(body);
    NurbsSurface cap_base; cap_base.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
    cap_base.set_cv(0, 0, Point(-radius, -radius, 0));
    cap_base.set_cv(1, 0, Point(radius, -radius, 0));
    cap_base.set_cv(0, 1, Point(-radius, radius, 0));
    cap_base.set_cv(1, 1, Point(radius, radius, 0));
    int si_base = brep.add_surface(cap_base);

    int fi_body = brep.add_face(si_body, false);
    int li_body = brep.add_loop(fi_body, BRepLoopType::Outer);
    auto c2d_base = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.first, 0), Point(dom_u.second, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_base), ei_base, li_body, false, BRepTrimType::Mated);
    auto c2d_sr = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.first, 0), Point(dom_u.second, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li_body, false, BRepTrimType::Seam);
    auto c2d_apex = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.second, 0), Point(dom_u.first, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_apex), -1, li_body, false, BRepTrimType::Singular);
    auto c2d_sl = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.second, 0), Point(dom_u.first, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_sl), ei_seam, li_body, true, BRepTrimType::Seam);

    const double cw = std::sqrt(2.0) / 2.0;
    double ccx[] = {1,1,0,-1,-1,-1,0,1,1}, ccy[] = {0,1,1,1,0,-1,-1,-1,0};
    double cwt[] = {1,cw,1,cw,1,cw,1,cw,1}, ckn[] = {0,0,1,1,2,2,3,3,4,4};
    auto make_cap_circle = [&]() {
        NurbsCurve c(3, true, 3, 9);
        for (int i = 0; i < 10; i++) c.set_nurbsknot(i, ckn[i]);
        for (int i = 0; i < 9; i++)
            c.set_cv_4d(i, (0.5+0.5*ccx[i])*cwt[i], (0.5+0.5*ccy[i])*cwt[i], 0.0, cwt[i]);
        return c;
    };
    int fi_base = brep.add_face(si_base, true);
    int li_base = brep.add_loop(fi_base, BRepLoopType::Outer);
    brep.add_trim(brep.add_curve_2d(make_cap_circle()), ei_base, li_base, true, BRepTrimType::Mated);

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::create_torus(double major_radius, double minor_radius) {
    // Torus: a single closed face, periodic in BOTH u (major circle) and v (minor circle). No caps,
    // no poles -- two seams: the u-seam (minor circle at u=0) and the v-seam (outer major circle at
    // v=0), meeting at one corner vertex. The loop is the UV rectangle [0,4]x[0,4] = 4 Seam trims.
    BRep brep;
    brep.name = "torus";
    NurbsSurface body = Primitives::torus_surface(0, 0, 0, major_radius, minor_radius);
    auto dom_u = body.domain(0);
    auto dom_v = body.domain(1);

    Point p_corner = body.point_at_corner(0, 0);   // (major+minor, 0, 0)
    int vi = brep.add_vertex(p_corner);
    brep.m_topology_vertices.push_back({vi, {}});

    // u-seam: v -> point_at(u0, v), the minor circle at u=0. v-seam: u -> point_at(u, v0), outer
    // major circle at v=0. Sample each as a closed polyline (the exact rational pcurve is a circle).
    auto iso_curve = [&](bool along_v) {
        std::vector<Point> pts; int n = 64;
        for (int i = 0; i <= n; ++i) {
            double t = (double)i / n;
            Point p = along_v ? body.point_at(dom_u.first, dom_v.first + (dom_v.second-dom_v.first)*t)
                              : body.point_at(dom_u.first + (dom_u.second-dom_u.first)*t, dom_v.first);
            pts.push_back(p);
        }
        return NurbsCurve::create(false, 3, pts);
    };
    int ci_useam = brep.add_curve_3d(iso_curve(true));    // minor circle (varies v)
    int ci_vseam = brep.add_curve_3d(iso_curve(false));   // major circle (varies u)
    int ei_useam = brep.add_edge(ci_useam, vi, vi);
    int ei_vseam = brep.add_edge(ci_vseam, vi, vi);

    int si_body = brep.add_surface(body);
    int fi_body = brep.add_face(si_body, false);
    int li_body = brep.add_loop(fi_body, BRepLoopType::Outer);
    // Loop around the UV rectangle: bottom(v-seam), right(u-seam), top(v-seam), left(u-seam).
    auto c2d_bottom = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.first, 0), Point(dom_u.second, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_bottom), ei_vseam, li_body, false, BRepTrimType::Seam);
    auto c2d_right = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.first, 0), Point(dom_u.second, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_right), ei_useam, li_body, false, BRepTrimType::Seam);
    auto c2d_top = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.second, 0), Point(dom_u.first, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_top), ei_vseam, li_body, true, BRepTrimType::Seam);
    auto c2d_left = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.second, 0), Point(dom_u.first, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_left), ei_useam, li_body, true, BRepTrimType::Seam);

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::create_block_with_hole(double sx, double sy, double sz, double hole_radius) {
    BRep brep;
    brep.name = "block_with_hole";
    double hx = sx * 0.5, hy = sy * 0.5, hz = sz * 0.5;

    // 8 box corners
    Point corners[8] = {
        Point(-hx, -hy, -hz),
        Point( hx, -hy, -hz),
        Point( hx,  hy, -hz),
        Point(-hx,  hy, -hz),
        Point(-hx, -hy,  hz),
        Point( hx, -hy,  hz),
        Point( hx,  hy,  hz),
        Point(-hx,  hy,  hz),
    };
    for (int i = 0; i < 8; ++i) brep.add_vertex(corners[i]);

    // Topology vertices for box corners
    for (int i = 0; i < 8; ++i) {
        BRepVertex tv; tv.point_index = i;
        brep.m_topology_vertices.push_back(tv);
    }

    // 12 box edges
    int edge_verts[12][2] = {
        {0,1},{1,2},{2,3},{3,0},  // bottom
        {4,5},{5,6},{6,7},{7,4},  // top
        {0,4},{1,5},{2,6},{3,7},  // vertical
    };
    for (int i = 0; i < 12; ++i) {
        auto line = NurbsCurve::create(false, 1, {corners[edge_verts[i][0]], corners[edge_verts[i][1]]});
        brep.add_curve_3d(line);
    }
    for (int i = 0; i < 12; ++i)
        brep.add_edge(i, edge_verts[i][0], edge_verts[i][1]);

    // 4 side walls (full-boundary, no hole interaction)
    int side_faces[4][4] = {
        {0, 1, 5, 4}, // front
        {1, 2, 6, 5}, // right
        {2, 3, 7, 6}, // back
        {3, 0, 4, 7}, // left
    };
    auto find_edge = [&](int v0, int v1) -> int {
        for (int e = 0; e < 12; ++e) {
            if ((edge_verts[e][0] == v0 && edge_verts[e][1] == v1) ||
                (edge_verts[e][0] == v1 && edge_verts[e][1] == v0))
                return e;
        }
        return -1;
    };
    for (int fi = 0; fi < 4; ++fi) {
        int* fv = side_faces[fi];
        Point p00 = corners[fv[0]], p10 = corners[fv[1]];
        Point p01 = corners[fv[3]], p11 = corners[fv[2]];
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, p00); srf.set_cv(1, 0, p10);
        srf.set_cv(0, 1, p01); srf.set_cv(1, 1, p11);
        int si = brep.add_surface(srf);
        int face_idx = brep.add_face(si, false);
        int loop_idx = brep.add_loop(face_idx, BRepLoopType::Outer);
        Point uv[4] = {
            Point(0, 0, 0),
            Point(1, 0, 0),
            Point(1, 1, 0),
            Point(0, 1, 0),
        };
        for (int ei = 0; ei < 4; ++ei) {
            int next = (ei + 1) % 4;
            auto tc = NurbsCurve::create(false, 1, {uv[ei], uv[next]});
            int c2d = brep.add_curve_2d(tc);
            int edge_idx = find_edge(fv[ei], fv[next]);
            bool rev = (edge_verts[edge_idx][0] != fv[ei]);
            brep.add_trim(c2d, edge_idx, loop_idx, rev, BRepTrimType::Mated);
        }
    }

    // Cylinder hole surface (through-hole in Z at center)
    NurbsSurface cyl_srf = Primitives::cylinder_surface(0, 0, -hz, hole_radius, sz);
    auto dom_u = cyl_srf.domain(0);
    auto dom_v = cyl_srf.domain(1);
    int si_cyl = brep.add_surface(cyl_srf);
    int fi_cyl = brep.add_face(si_cyl, true); // reversed so normals point inward
    int li_cyl = brep.add_loop(fi_cyl, BRepLoopType::Outer);

    // Hole 3D edge curves (circles at bottom and top)
    NurbsCurve circle_bot = Primitives::circle(0, 0, -hz, hole_radius);
    NurbsCurve circle_top = Primitives::circle(0, 0, hz, hole_radius);
    NurbsCurve seam_line = NurbsCurve::create(false, 1, {
        Point(hole_radius, 0, -hz),
        Point(hole_radius, 0, hz),
    });
    int ci_bot = brep.add_curve_3d(circle_bot);
    int ci_top = brep.add_curve_3d(circle_top);
    int ci_seam = brep.add_curve_3d(seam_line);

    // Add vertices for seam points
    int vi_seam_bot = brep.add_vertex(Point(hole_radius, 0, -hz));
    int vi_seam_top = brep.add_vertex(Point(hole_radius, 0, hz));
    brep.m_topology_vertices.push_back({vi_seam_bot, {}});
    brep.m_topology_vertices.push_back({vi_seam_top, {}});

    int ei_bot = brep.add_edge(ci_bot, 8, 8);   // closed edge
    int ei_top = brep.add_edge(ci_top, 9, 9);
    int ei_seam = brep.add_edge(ci_seam, 8, 9);

    // 4 trims for cylinder body (same pattern as create_cylinder)
    auto c2d_bot = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.first, 0),
        Point(dom_u.second, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_bot), ei_bot, li_cyl, false, BRepTrimType::Mated);
    auto c2d_sr = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.first, 0),
        Point(dom_u.second, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li_cyl, false, BRepTrimType::Seam);
    auto c2d_top = NurbsCurve::create(false, 1, {
        Point(dom_u.second, dom_v.second, 0),
        Point(dom_u.first, dom_v.second, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_top), ei_top, li_cyl, true, BRepTrimType::Mated);
    auto c2d_sl = NurbsCurve::create(false, 1, {
        Point(dom_u.first, dom_v.second, 0),
        Point(dom_u.first, dom_v.first, 0),
    });
    brep.add_trim(brep.add_curve_2d(c2d_sl), ei_seam, li_cyl, true, BRepTrimType::Seam);

    // Bottom and top faces: planar with circular hole
    auto make_cap = [&](double z, bool reversed, int circle_edge_idx) {
        double r = std::max(hx, hy);
        NurbsSurface cap;
        cap.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        cap.set_cv(0, 0, Point(-r, -r, z)); cap.set_cv(1, 0, Point(r, -r, z));
        cap.set_cv(0, 1, Point(-r, r, z));   cap.set_cv(1, 1, Point(r, r, z));
        int si = brep.add_surface(cap);
        int fi = brep.add_face(si, reversed);

        // Outer loop: 4 box edges matching this cap
        int outer_li = brep.add_loop(fi, BRepLoopType::Outer);
        int* fv = (z < 0) ? new int[4]{0, 3, 2, 1} : new int[4]{4, 5, 6, 7};
        // Map box corners to UV on this cap surface
        for (int ei = 0; ei < 4; ++ei) {
            int next = (ei + 1) % 4;
            double u0 = (corners[fv[ei]][0] + r) / (2.0 * r);
            double v0 = (corners[fv[ei]][1] + r) / (2.0 * r);
            double u1 = (corners[fv[next]][0] + r) / (2.0 * r);
            double v1 = (corners[fv[next]][1] + r) / (2.0 * r);
            auto tc = NurbsCurve::create(false, 1, {
                Point(u0, v0, 0),
                Point(u1, v1, 0),
            });
            int c2d = brep.add_curve_2d(tc);
            int edge_idx = find_edge(fv[ei], fv[next]);
            bool rev = (edge_verts[edge_idx][0] != fv[ei]);
            brep.add_trim(c2d, edge_idx, outer_li, rev, BRepTrimType::Mated);
        }
        delete[] fv;

        // Inner loop: circular hole trim in UV
        int inner_li = brep.add_loop(fi, BRepLoopType::Inner);
        const double cw = std::sqrt(2.0) / 2.0;
        double ccx[] = {1,1,0,-1,-1,-1,0,1,1}, ccy[] = {0,1,1,1,0,-1,-1,-1,0};
        double cwt[] = {1,cw,1,cw,1,cw,1,cw,1}, ckn[] = {0,0,1,1,2,2,3,3,4,4};
        NurbsCurve hole_crv(3, true, 3, 9);
        for (int i = 0; i < 10; ++i) hole_crv.set_nurbsknot(i, ckn[i]);
        double cr = hole_radius / (2.0 * r); // radius in UV
        double cx_uv = 0.5, cy_uv = 0.5;     // center in UV
        for (int i = 0; i < 9; ++i)
            hole_crv.set_cv_4d(i, (cx_uv + cr*ccx[i])*cwt[i], (cy_uv + cr*ccy[i])*cwt[i], 0.0, cwt[i]);
        brep.add_trim(brep.add_curve_2d(hole_crv), circle_edge_idx, inner_li,
                       reversed, BRepTrimType::Mated);
    };

    make_cap(-hz, true, ei_bot);
    make_cap(hz, false, ei_top);

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::from_polylines(const std::vector<Polyline>& polylines) {
    BRep brep;
    brep.name = "polysurface";
    double tol = 1e-6;

    auto find_or_add = [&](const Point& p) -> int {
        for (int i = 0; i < (int)brep.m_vertices.size(); ++i) {
            double dx = p[0] - brep.m_vertices[i][0];
            double dy = p[1] - brep.m_vertices[i][1];
            double dz = p[2] - brep.m_vertices[i][2];
            if (dx*dx + dy*dy + dz*dz < tol*tol) return i;
        }
        int idx = brep.add_vertex(p);
        brep.m_topology_vertices.push_back({idx, {}});
        return idx;
    };

    std::vector<std::vector<int>> poly_vi;
    for (const auto& pl : polylines) {
        auto pts = pl.get_points();
        int n = pl.is_closed() ? (int)pts.size() - 1 : (int)pts.size();
        std::vector<int> vi;
        for (int i = 0; i < n; ++i) vi.push_back(find_or_add(pts[i]));
        poly_vi.push_back(vi);
    }

    std::map<std::pair<int,int>, int> edge_map;
    auto get_edge = [&](int v0, int v1) -> std::pair<int, bool> {
        int lo = std::min(v0, v1), hi = std::max(v0, v1);
        auto it = edge_map.find({lo, hi});
        if (it != edge_map.end()) return {it->second, v0 != lo};
        auto line = NurbsCurve::create(false, 1, {brep.m_vertices[v0], brep.m_vertices[v1]});
        int ci = brep.add_curve_3d(line);
        int ei = brep.add_edge(ci, lo, hi);
        edge_map[{lo, hi}] = ei;
        return {ei, v0 != lo};
    };

    for (int pi = 0; pi < (int)polylines.size(); ++pi) {
        const auto& vi = poly_vi[pi];
        int n = (int)vi.size();
        if (n < 3) continue;

        Point org; Plane plane;
        polylines[pi].get_fast_plane(org, plane);
        if (!plane.is_valid()) continue;
        Vector xa = plane.x_axis(), ya = plane.y_axis();

        std::vector<double> us(n), vs(n);
        double umin = 1e30, umax = -1e30, vmin = 1e30, vmax = -1e30;
        for (int i = 0; i < n; ++i) {
            double dx = brep.m_vertices[vi[i]][0] - org[0];
            double dy = brep.m_vertices[vi[i]][1] - org[1];
            double dz = brep.m_vertices[vi[i]][2] - org[2];
            us[i] = dx*xa[0] + dy*xa[1] + dz*xa[2];
            vs[i] = dx*ya[0] + dy*ya[1] + dz*ya[2];
            umin = std::min(umin, us[i]); umax = std::max(umax, us[i]);
            vmin = std::min(vmin, vs[i]); vmax = std::max(vmax, vs[i]);
        }
        double pad = std::max(umax - umin, vmax - vmin) * 0.01;
        umin -= pad; umax += pad; vmin -= pad; vmax += pad;
        double du = umax - umin, dv = vmax - vmin;

        auto pt3d = [&](double u, double v) {
            return Point(org[0] + u*xa[0] + v*ya[0],
                         org[1] + u*xa[1] + v*ya[1],
                         org[2] + u*xa[2] + v*ya[2]);
        };
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, pt3d(umin, vmin)); srf.set_cv(1, 0, pt3d(umax, vmin));
        srf.set_cv(0, 1, pt3d(umin, vmax)); srf.set_cv(1, 1, pt3d(umax, vmax));
        int si = brep.add_surface(srf);

        int fi = brep.add_face(si, false);
        int li = brep.add_loop(fi, BRepLoopType::Outer);

        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            double u0 = (us[i] - umin) / du, v0 = (vs[i] - vmin) / dv;
            double u1 = (us[j] - umin) / du, v1 = (vs[j] - vmin) / dv;
            auto tc = NurbsCurve::create(false, 1, {
                Point(u0, v0, 0),
                Point(u1, v1, 0),
            });
            int c2d = brep.add_curve_2d(tc);
            auto [ei, rev] = get_edge(vi[i], vi[j]);
            BRepTrimType type = brep.m_topology_edges[ei].trim_indices.empty()
                ? BRepTrimType::Boundary : BRepTrimType::Mated;
            for (int ti : brep.m_topology_edges[ei].trim_indices)
                brep.m_trims[ti].type = BRepTrimType::Mated;
            brep.add_trim(c2d, ei, li, rev, type);
        }
    }

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

BRep BRep::from_nurbscurves(const std::vector<NurbsCurve>& curves, const std::vector<std::vector<NurbsCurve>>& holes) {
    BRep brep;
    brep.name = "polysurface";
    double tol = 1e-6;

    auto find_or_add = [&](const Point& p) -> int {
        for (int i = 0; i < (int)brep.m_vertices.size(); ++i) {
            double dx = p[0] - brep.m_vertices[i][0];
            double dy = p[1] - brep.m_vertices[i][1];
            double dz = p[2] - brep.m_vertices[i][2];
            if (dx*dx + dy*dy + dz*dz < tol*tol) return i;
        }
        int idx = brep.add_vertex(p);
        brep.m_topology_vertices.push_back({idx, {}});
        return idx;
    };

    auto project_curve_to_uv = [&](const NurbsCurve& crv, const Point& org,
                                    const Vector& xa, const Vector& ya,
                                    double umin, double vmin, double du, double dv) -> NurbsCurve {
        NurbsCurve crv2d(3, crv.is_rational(), crv.order(), crv.cv_count());
        for (int i = 0; i < crv.nurbsknot_count(); ++i) crv2d.set_nurbsknot(i, crv.nurbsknot(i));
        for (int i = 0; i < crv.cv_count(); ++i) {
            if (crv.is_rational()) {
                auto [wx, wy, wz, w] = crv.get_cv_4d(i);
                double x = wx/w, y = wy/w, z = wz/w;
                double dx = x-org[0], dy = y-org[1], dz = z-org[2];
                double u = (dx*xa[0]+dy*xa[1]+dz*xa[2] - umin) / du;
                double v = (dx*ya[0]+dy*ya[1]+dz*ya[2] - vmin) / dv;
                crv2d.set_cv_4d(i, u*w, v*w, 0.0, w);
            } else {
                Point cv = crv.get_cv(i);
                double dx = cv[0]-org[0], dy = cv[1]-org[1], dz = cv[2]-org[2];
                double u = (dx*xa[0]+dy*xa[1]+dz*xa[2] - umin) / du;
                double v = (dx*ya[0]+dy*ya[1]+dz*ya[2] - vmin) / dv;
                crv2d.set_cv(i, Point(u, v, 0));
            }
        }
        return crv2d;
    };

    auto add_curve_loop = [&](const NurbsCurve& crv, int face_idx, BRepLoopType loop_type,
                               const Point& org, const Vector& xa, const Vector& ya,
                               double umin, double vmin, double du, double dv) {
        int li = brep.add_loop(face_idx, loop_type);
        int ci3d = brep.add_curve_3d(crv);
        NurbsCurve crv2d = project_curve_to_uv(crv, org, xa, ya, umin, vmin, du, dv);
        int c2d = brep.add_curve_2d(crv2d);
        Point sp = crv.point_at(crv.domain().first);
        Point ep = crv.point_at(crv.domain().second);
        int vi_s = find_or_add(sp), vi_e = crv.is_closed() ? vi_s : find_or_add(ep);
        int lo = std::min(vi_s, vi_e), hi = std::max(vi_s, vi_e);
        int ei = brep.add_edge(ci3d, lo, hi);
        brep.add_trim(c2d, ei, li, false, BRepTrimType::Boundary);
    };

    auto cv_points = [](const NurbsCurve& c) -> std::vector<Point> {
        std::vector<Point> pts;
        int nc = c.cv_count();
        pts.reserve(nc);
        for (int k = 0; k < nc; ++k) {
            if (c.is_rational()) {
                auto [wx, wy, wz, w] = c.get_cv_4d(k);
                if (w != 0.0) pts.emplace_back(wx/w, wy/w, wz/w);
            } else {
                pts.push_back(c.get_cv(k));
            }
        }
        return pts;
    };

    for (int ci = 0; ci < (int)curves.size(); ++ci) {
        const auto& crv = curves[ci];
        auto pts = cv_points(crv);
        // drop the repeated last point if closed (first == last)
        if (pts.size() >= 2) {
            auto& f = pts.front(); auto& b = pts.back();
            double dd = (f[0]-b[0])*(f[0]-b[0])+(f[1]-b[1])*(f[1]-b[1])+(f[2]-b[2])*(f[2]-b[2]);
            if (dd < tol*tol) pts.pop_back();
        }
        int n = (int)pts.size();
        if (n < 3) continue;

        Polyline pl(pts);
        Point org; Plane plane;
        pl.get_fast_plane(org, plane);
        if (!plane.is_valid()) continue;
        Vector xa = plane.x_axis(), ya = plane.y_axis();

        std::vector<double> us(n), vs(n);
        double umin = 1e30, umax = -1e30, vmin = 1e30, vmax = -1e30;
        for (int i = 0; i < n; ++i) {
            double dx = pts[i][0] - org[0], dy = pts[i][1] - org[1], dz = pts[i][2] - org[2];
            us[i] = dx*xa[0] + dy*xa[1] + dz*xa[2];
            vs[i] = dx*ya[0] + dy*ya[1] + dz*ya[2];
            umin = std::min(umin, us[i]); umax = std::max(umax, us[i]);
            vmin = std::min(vmin, vs[i]); vmax = std::max(vmax, vs[i]);
        }
        // Include hole CVs in bounds (convex hull property guarantees curve lies within)
        if (ci < (int)holes.size()) {
            for (const auto& hcrv : holes[ci]) {
                for (const auto& hp : cv_points(hcrv)) {
                    double dx = hp[0]-org[0], dy = hp[1]-org[1], dz = hp[2]-org[2];
                    double hu = dx*xa[0]+dy*xa[1]+dz*xa[2], hv = dx*ya[0]+dy*ya[1]+dz*ya[2];
                    umin = std::min(umin, hu); umax = std::max(umax, hu);
                    vmin = std::min(vmin, hv); vmax = std::max(vmax, hv);
                }
            }
        }
        double pad = std::max(umax - umin, vmax - vmin) * 0.01;
        umin -= pad; umax += pad; vmin -= pad; vmax += pad;
        double du = umax - umin, dv = vmax - vmin;

        auto pt3d = [&](double u, double v) {
            return Point(org[0]+u*xa[0]+v*ya[0], org[1]+u*xa[1]+v*ya[1], org[2]+u*xa[2]+v*ya[2]);
        };
        NurbsSurface srf;
        srf.create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0);
        srf.set_cv(0, 0, pt3d(umin, vmin)); srf.set_cv(1, 0, pt3d(umax, vmin));
        srf.set_cv(0, 1, pt3d(umin, vmax)); srf.set_cv(1, 1, pt3d(umax, vmax));
        int si = brep.add_surface(srf);
        int fi = brep.add_face(si, false);

        add_curve_loop(crv, fi, BRepLoopType::Outer, org, xa, ya, umin, vmin, du, dv);

        if (ci < (int)holes.size()) {
            for (const auto& hcrv : holes[ci]) {
                add_curve_loop(hcrv, fi, BRepLoopType::Inner, org, xa, ya, umin, vmin, du, dv);
            }
        }
    }

    for (int ei = 0; ei < (int)brep.m_topology_edges.size(); ++ei) {
        brep.m_topology_vertices[brep.m_topology_edges[ei].start_vertex].edge_indices.push_back(ei);
        brep.m_topology_vertices[brep.m_topology_edges[ei].end_vertex].edge_indices.push_back(ei);
    }
    return brep;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////////////////

int BRep::face_count() const { return (int)m_faces.size(); }
int BRep::edge_count() const { return (int)m_topology_edges.size(); }
int BRep::vertex_count() const { return (int)m_vertices.size(); }

bool BRep::is_valid() const {
    if (m_faces.empty() || m_surfaces.empty() || m_vertices.empty()) return false;
    for (const auto& f : m_faces)
        if (f.surface_index < 0 || f.surface_index >= (int)m_surfaces.size()) return false;
    for (const auto& l : m_loops)
        if (l.face_index < 0 || l.face_index >= (int)m_faces.size()) return false;
    for (const auto& t : m_trims) {
        if (t.curve_2d_index < 0 || t.curve_2d_index >= (int)m_curves_2d.size()) return false;
        if (t.loop_index < 0 || t.loop_index >= (int)m_loops.size()) return false;
    }
    for (const auto& e : m_topology_edges) {
        if (e.start_vertex < 0 || e.start_vertex >= (int)m_topology_vertices.size()) return false;
        if (e.end_vertex < 0 || e.end_vertex >= (int)m_topology_vertices.size()) return false;
    }
    return true;
}

bool BRep::is_solid() const {
    if (m_topology_edges.empty()) return false;
    // bbox diagonal for the degenerate-edge tolerance.
    double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
    for (const auto& p : m_vertices) {
        xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
        xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
    }
    double diag = (m_vertices.empty()) ? 1.0 :
        std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
    double deg_tol = std::max(diag * 1e-7, 1e-12);
    for (const auto& e : m_topology_edges) {
        if ((int)e.trim_indices.size() == 2) continue;
        // A DEGENERATE edge (3D curve collapsed to a point, e.g. a sphere/cone pole) is
        // watertight by construction -- the pole is a single point fully enclosed by its
        // face -- and OCCT excludes such degenerate edges from manifold checks. Skip them;
        // only genuine (non-zero-length) edges must be 2-trim.
        int ci = e.curve_3d_index;
        if (ci >= 0 && ci < (int)m_curves_3d.size()) {
            const NurbsCurve& c = m_curves_3d[ci];
            auto dc = c.domain();
            Point p0 = c.point_at(dc.first);
            double ext = 0.0;
            for (int k = 1; k <= 4; ++k) {
                Point pk = c.point_at(dc.first + (dc.second - dc.first) * k / 4.0);
                ext = std::max(ext, p0.distance(pk));
            }
            if (ext < deg_tol) continue;
        }
        return false;
    }
    return true;
}

bool BRep::contains_point(const Mesh& boundary, const Point& p) const {
    // Ray-cast parity: cast a ray from p in an irregular direction to a far point and
    // count boundary crossings. Odd => inside. The irregular direction avoids grazing
    // shared edges/vertices that would mis-count.
    const double dx = 0.5773502691, dy = 0.6539124, dz = 0.5023147;  // irregular, ~unit
    const double big = 1e6;
    Line ray(p[0], p[1], p[2], p[0] + dx*big, p[1] + dy*big, p[2] + dz*big);
    auto hits = Intersection::ray_mesh(ray, boundary, 1e-9, true);
    return (hits.size() % 2) == 1;
}

bool BRep::contains_point(const Point& p) const {
    return contains_point(mesh(), p);
}

double BRep::volume() const {
    // Divergence theorem: V = (1/3) sum_faces flux_outward, flux = integral of S . n_out dA.
    // This is made independent of stored orientation flags (which differ between primitive
    // constructors and the boolean rebuilder): each face's OUTWARD sign is determined
    // geometrically (step off the face along its natural normal and test inside/outside),
    // and trimmed areas/fluxes are computed with the surface's NATURAL orientation.
    //  - Planar faces: flux_nat = (Q . N_nat) * area, area = |outer loop| - sum|inner loop|
    //    (each loop area is the magnitude of the lifted boundary integral 1/2 |C x C'|).
    //  - Curved faces: composite Gauss of S . (S_u x S_v) over the trim's UV bounding box
    //    (exact for axis-aligned/rectangular trims, e.g. cylinder z-cuts and full domains).
    static const double GN[5] = {-0.9061798459386640, -0.5384693101056831, 0.0,
                                  0.5384693101056831, 0.9061798459386640};
    static const double GW[5] = {0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
                                 0.4786286704993665, 0.2369268850561891};
    auto cross = [](const Vector& a, const Vector& b) {
        return Vector(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]);
    };
    auto is_planar = [&](const NurbsSurface& s) {
        auto [u0,u1] = s.domain(0); auto [v0,v1] = s.domain(1);
        Vector n0 = s.normal_at(u0 + (u1-u0)*0.5, v0 + (v1-v0)*0.5);
        double uu[3] = {0.25, 0.5, 0.75}, vv[3] = {0.3, 0.6, 0.8};
        for (int i = 0; i < 3; i++) {
            Vector n = s.normal_at(u0 + (u1-u0)*uu[i], v0 + (v1-v0)*vv[i]);
            if (cross(n0, n).magnitude() > 1e-7) return false;
        }
        return true;
    };
    // Lifted boundary vector-area of one loop: 1/2 * closed integral C x C'. Convention-
    // independent: the loop's pcurves are chained head-to-tail by matching UV endpoints
    // (each used forward or reversed as the chain requires), so the stored trim.reversed
    // flags are not relied upon. Exact for exact pcurves (e.g. NURBS circles).
    auto loop_vector_area = [&](const NurbsSurface& srf, const BRepLoop& loop) {
        struct Seg { const NurbsCurve* pc; double t0, t1; Point ps, pe; };
        std::vector<Seg> segs;
        for (int ti : loop.trim_indices) {
            if (ti < 0 || ti >= (int)m_trims.size()) continue;
            int c2 = m_trims[ti].curve_2d_index;
            if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
            const NurbsCurve& pc = m_curves_2d[c2];
            auto dc = pc.domain();
            segs.push_back({&pc, dc.first, dc.second, pc.point_at(dc.first), pc.point_at(dc.second)});
        }
        if (segs.empty()) return Vector(0,0,0);
        // Chain by nearest-endpoint greedy walk; each entry is (index, forward?).
        std::vector<std::pair<int,bool>> order;
        std::vector<bool> used(segs.size(), false);
        order.push_back({0, true}); used[0] = true;
        Point tail = segs[0].pe;
        auto d2 = [](const Point& a, const Point& b){ double dx=a[0]-b[0],dy=a[1]-b[1]; return dx*dx+dy*dy; };
        for (size_t k = 1; k < segs.size(); ++k) {
            int best = -1; bool fwd = true; double bd = 1e300;
            for (size_t j = 0; j < segs.size(); ++j) {
                if (used[j]) continue;
                double ds = d2(segs[j].ps, tail), de = d2(segs[j].pe, tail);
                if (ds < bd) { bd = ds; best = (int)j; fwd = true; }
                if (de < bd) { bd = de; best = (int)j; fwd = false; }
            }
            if (best < 0) break;
            used[best] = true; order.push_back({best, fwd});
            tail = fwd ? segs[best].pe : segs[best].ps;
        }
        Vector acc(0,0,0);
        const int NS = 24;
        for (auto& od : order) {
            const Seg& sg = segs[od.first];
            bool fwd = od.second;
            for (int s = 0; s < NS; s++) {
                double a = sg.t0 + (sg.t1-sg.t0)*s/NS, b = sg.t0 + (sg.t1-sg.t0)*(s+1)/NS;
                double mid = 0.5*(a+b), half = 0.5*(b-a);
                for (int g = 0; g < 5; g++) {
                    double t = mid + half*GN[g];
                    auto pe = sg.pc->evaluate(t, 1);
                    if (pe.size() < 2) continue;
                    const Vector& uv = pe[0]; Vector duv = pe[1];
                    if (!fwd) duv = Vector(-duv[0], -duv[1], -duv[2]);
                    auto se = srf.evaluate(uv[0], uv[1], 1);
                    if (se.size() < 3) continue;
                    const Vector& S = se[0]; const Vector& Sv = se[1]; const Vector& Su = se[2];
                    Vector Cp(Su[0]*duv[0]+Sv[0]*duv[1], Su[1]*duv[0]+Sv[1]*duv[1], Su[2]*duv[0]+Sv[2]*duv[1]);
                    Vector cr = cross(S, Cp);
                    double w = GW[g]*half;
                    acc = Vector(acc[0]+w*cr[0], acc[1]+w*cr[1], acc[2]+w*cr[2]);
                }
            }
        }
        return Vector(0.5*acc[0], 0.5*acc[1], 0.5*acc[2]);
    };
    // Interior UV point of a face (CDT centroid), plus its natural unit normal.
    auto face_interior = [&](const BRepFace& face, const NurbsSurface& srf, Point& P3, Vector& Nnat) {
        std::vector<Polyline> outers, inners;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            std::vector<Point> pts;
            for (int ti : m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int c2 = m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& pc = m_curves_2d[c2];
                auto dc = pc.domain();
                int n = std::max(pc.cv_count()*3, 6);
                for (int i = 0; i < n; ++i) {
                    Point uv = pc.point_at(dc.first + (dc.second-dc.first)*i/n);
                    pts.push_back(Point(uv[0], uv[1], 0));
                }
            }
            if (pts.size() < 3) continue;
            if (m_loops[li].type == BRepLoopType::Outer) outers.emplace_back(pts);
            else inners.emplace_back(pts);
        }
        auto [u0,u1] = srf.domain(0); auto [v0,v1] = srf.domain(1);
        double cu = 0.5*(u0+u1), cv = 0.5*(v0+v1);
        auto pip = [&](double uu, double vv, const Polyline& poly) -> bool {
            auto pp = poly.get_points();
            bool inside = false; int j = (int)pp.size() - 1;
            for (int i = 0; i < (int)pp.size(); ++i) {
                double denom = (pp[j][1] - pp[i][1]); if (denom == 0.0) denom = 1e-300;
                if (((pp[i][1] > vv) != (pp[j][1] > vv)) &&
                    (uu < (pp[j][0]-pp[i][0])*(vv-pp[i][1])/denom + pp[i][0]))
                    inside = !inside;
                j = i;
            }
            return inside;
        };
        auto in_material = [&](double uu, double vv) -> bool {
            if (!outers.empty()) {
                bool any = false;
                for (auto& op : outers) if (pip(uu, vv, op)) { any = true; break; }
                if (!any) return false;
            }
            for (auto& ip : inners) if (pip(uu, vv, ip)) return false;
            return true;
        };
        if (!outers.empty()) {
            std::vector<Polyline> all; all.push_back(outers[0]);
            for (auto& in : inners) all.push_back(in);
            std::vector<std::array<int,3>> tris;
            try { tris = RemeshCDT::triangulate(all); } catch (...) {}
            std::vector<Point> flat;
            for (auto& pl : all) { auto pp = pl.get_points(); for (auto& p : pp) flat.push_back(p); }
            // Largest-area triangle whose centroid is on the face MATERIAL (inside the outer
            // loop, outside every hole). The first triangle / domain centre can land in a hole
            // (e.g. an annulus' centre) -> a wrong outward-sign probe -> wrong flux.
            double best_a = -1.0;
            for (const auto& t : tris) {
                if (t[0] >= (int)flat.size() || t[1] >= (int)flat.size() || t[2] >= (int)flat.size()) continue;
                const auto& a = flat[t[0]]; const auto& b = flat[t[1]]; const auto& c = flat[t[2]];
                double tcu = (a[0]+b[0]+c[0])/3.0, tcv = (a[1]+b[1]+c[1])/3.0;
                double ar = std::abs((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]));
                if (ar > best_a && in_material(tcu, tcv)) { best_a = ar; cu = tcu; cv = tcv; }
            }
            if (best_a < 0.0) {
                // CDT gave nothing usable -> grid-sample for any in-material point.
                bool found = false;
                for (int iu = 1; iu < 12 && !found; ++iu)
                    for (int iv = 1; iv < 12 && !found; ++iv) {
                        double su = u0 + (u1-u0)*iu/12.0, sv = v0 + (v1-v0)*iv/12.0;
                        if (in_material(su, sv)) { cu = su; cv = sv; found = true; }
                    }
            }
        }
        P3 = srf.point_at(cu, cv);
        Nnat = srf.normal_at(cu, cv);  // unit; natural orientation = Su x Sv direction
    };

    Mesh bmesh = mesh();
    // bbox diagonal for the off-face step.
    double xmin=1e300,ymin=1e300,zmin=1e300,xmax=-1e300,ymax=-1e300,zmax=-1e300;
    for (const auto& p : m_vertices) {
        xmin=std::min(xmin,p[0]); ymin=std::min(ymin,p[1]); zmin=std::min(zmin,p[2]);
        xmax=std::max(xmax,p[0]); ymax=std::max(ymax,p[1]); zmax=std::max(zmax,p[2]);
    }
    double diag = std::sqrt((xmax-xmin)*(xmax-xmin)+(ymax-ymin)*(ymax-ymin)+(zmax-zmin)*(zmax-zmin));
    double eps = (diag > 0 ? diag : 1.0) * 1e-3;

    double total = 0.0;
    for (const auto& face : m_faces) {
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];

        Point P3; Vector Nnat;
        face_interior(face, srf, P3, Nnat);
        if (Nnat.magnitude() < 1e-12) continue;
        // Outward sign: stepping a little along +Nnat should leave the solid.
        Point probe(P3[0]+eps*Nnat[0], P3[1]+eps*Nnat[1], P3[2]+eps*Nnat[2]);
        double sign = contains_point(bmesh, probe) ? -1.0 : 1.0;

        double flux_nat = 0.0;
        bool curved_rect = false;
        if (is_planar(srf)) {
            double area = 0.0;
            for (int li : face.loop_indices) {
                if (li < 0 || li >= (int)m_loops.size()) continue;
                double la = loop_vector_area(srf, m_loops[li]).magnitude();
                area += (m_loops[li].type == BRepLoopType::Outer) ? la : -la;
            }
            // flux_nat = integral S . N_nat dA = (Q . N_nat) * area
            double qn = P3[0]*Nnat[0] + P3[1]*Nnat[1] + P3[2]*Nnat[2];
            flux_nat = qn * area;
        } else {
            // Trim UV bounding box AND the trim loops as UV polygons (so a non-rectangular trim --
            // a circular sphere cap, or a band with circular holes -- integrates over the actual
            // region, not the bbox). For a rectangular trim every Gauss point is inside, so this
            // reduces to the plain bbox quadrature (exact for cylinder bands).
            double umin=1e300,umax=-1e300,vmin=1e300,vmax=-1e300;
            std::vector<std::vector<std::array<double,2>>> outer_polys, inner_polys;
            for (int li : face.loop_indices) {
                if (li < 0 || li >= (int)m_loops.size()) continue;
                std::vector<std::array<double,2>> poly;
                for (int ti : m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)m_trims.size()) continue;
                    int c2 = m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                    const NurbsCurve& pc = m_curves_2d[c2];
                    auto dc = pc.domain();
                    int n = std::max(pc.cv_count()*4, 12);
                    for (int i = 0; i < n; ++i) {
                        Point uv = pc.point_at(dc.first + (dc.second-dc.first)*i/n);
                        umin=std::min(umin,uv[0]); umax=std::max(umax,uv[0]);
                        vmin=std::min(vmin,uv[1]); vmax=std::max(vmax,uv[1]);
                        poly.push_back({uv[0], uv[1]});
                    }
                }
                if (poly.size() >= 3) {
                    if (m_loops[li].type == BRepLoopType::Outer) outer_polys.push_back(poly);
                    else inner_polys.push_back(poly);
                }
            }
            if (umax <= umin || vmax <= vmin) { auto[a,b]=srf.domain(0); auto[c,d]=srf.domain(1); umin=a;umax=b;vmin=c;vmax=d; }
            auto in_poly = [](double u, double v, const std::vector<std::array<double,2>>& p) {
                bool inside = false;
                for (size_t i = 0, j = p.size()-1; i < p.size(); j = i++) {
                    if ((p[i][1] > v) != (p[j][1] > v) &&
                        u < (p[j][0]-p[i][0])*(v-p[i][1])/(p[j][1]-p[i][1]) + p[i][0])
                        inside = !inside;
                }
                return inside;
            };
            auto in_trim = [&](double u, double v) {
                bool ok = outer_polys.empty();  // no outer loop captured -> treat whole bbox as in
                for (auto& op : outer_polys) if (in_poly(u, v, op)) { ok = true; break; }
                if (!ok) return false;
                for (auto& ip : inner_polys) if (in_poly(u, v, ip)) return false;
                return true;
            };
            // A RECTANGULAR trim (cylinder band, full sphere) needs only NU=24 -- every Gauss
            // point is inside, so the quadrature is exact. A NON-rectangular trim (sphere caps,
            // a band with circular holes) has a curved mask boundary whose staircase error scales
            // ~1/NU, so use a finer grid there. (Sphere cap-cut faces are handled exactly below by
            // the analytic boundary-integral flux; this Gauss is the fallback for other curved faces.)
            auto [_du0,_du1] = srf.domain(0); auto [_dv0,_dv1] = srf.domain(1);
            bool rect_trim = inner_polys.empty()
                && std::abs(umin-_du0) < (_du1-_du0)*1e-3 && std::abs(umax-_du1) < (_du1-_du0)*1e-3
                && std::abs(vmin-_dv0) < (_dv1-_dv0)*1e-3 && std::abs(vmax-_dv1) < (_dv1-_dv0)*1e-3;
            curved_rect = rect_trim;
            int NU = rect_trim ? 24 : 96; int NV = NU;
            for (int iu = 0; iu < NU; iu++) {
                double ua=umin+(umax-umin)*iu/NU, ub=umin+(umax-umin)*(iu+1)/NU;
                double um=0.5*(ua+ub), uh=0.5*(ub-ua);
                for (int iv = 0; iv < NV; iv++) {
                    double va2=vmin+(vmax-vmin)*iv/NV, vb=vmin+(vmax-vmin)*(iv+1)/NV;
                    double vm=0.5*(va2+vb), vh=0.5*(vb-va2);
                    for (int gu = 0; gu < 5; gu++) {
                        double u=um+uh*GN[gu];
                        for (int gv = 0; gv < 5; gv++) {
                            double v=vm+vh*GN[gv];
                            if (!in_trim(u, v)) continue;
                            auto d = srf.evaluate(u, v, 1);
                            if (d.size() < 3) continue;
                            Vector nrm = cross(d[2], d[1]);  // Su x Sv (evaluate = [S, Sv, Su])
                            double integ = d[0][0]*nrm[0] + d[0][1]*nrm[1] + d[0][2]*nrm[2];
                            flux_nat += GW[gu]*GW[gv]*uh*vh*integ;
                        }
                    }
                }
            }
        }
        // Analytic sphere boundary-integral flux (exact, ~300x fewer surface evals than the masked
        // Gauss above; used for sphere cap-cut faces -- a sphere minus circular caps). Derivation:
        // flux_nat = integral P.(Su x Sv) du dv. With P = C + R*er (er radial) and Su x Sv || er,
        //   P.(Su x Sv) = C.(Su x Sv) + R^3 cos(phi) theta' phi'.
        // The first term integrates to C . (vector area) = C . (1/2) closed-integral P x dP (boundary,
        // exact). The second, via Green's in (theta,phi): R^3 integral cos(phi) dtheta dphi
        //   = -R^3 closed-integral sin(phi) dtheta = -R^2 closed-integral h dtheta, with h=(P-C).Zs and
        // theta = GEOMETRIC longitude atan2((P-C).Ys,(P-C).Xs) (seam-independent, OCCT EvalPnt2d style).
        // So flux_nat = sum_loops [ C.A_loop - R^2 * H_loop ] -- depends ONLY on the boundary curve's
        // 3D geometry, sidestepping the masked-Gauss region/staircase error.
        double flux_analytic = 0.0; bool have_analytic = false; bool pole_face = false;
        if (!is_planar(srf)) {
            auto [su0,su1]=srf.domain(0); auto [sv0,sv1]=srf.domain(1);
            double um=0.5*(su0+su1);
            Point Ps=srf.point_at(um,sv0), Pn=srf.point_at(um,sv1), Pm=srf.point_at(um,0.5*(sv0+sv1));
            Vector axis(Pn[0]-Ps[0],Pn[1]-Ps[1],Pn[2]-Ps[2]);
            double R = 0.5*axis.magnitude();
            if (R > 1e-9) {
                Point C(0.5*(Ps[0]+Pn[0]),0.5*(Ps[1]+Pn[1]),0.5*(Ps[2]+Pn[2]));
                Vector Zs(axis[0]/(2*R),axis[1]/(2*R),axis[2]/(2*R));
                // verify sphere: sample grid, |P-C| ~ R
                bool is_sphere=true;
                for(int i=0;i<=3&&is_sphere;++i)for(int j=0;j<=3&&is_sphere;++j){
                    Point p=srf.point_at(su0+(su1-su0)*i/3.0, sv0+(sv1-sv0)*j/3.0);
                    double rr=std::sqrt((p[0]-C[0])*(p[0]-C[0])+(p[1]-C[1])*(p[1]-C[1])+(p[2]-C[2])*(p[2]-C[2]));
                    if (std::abs(rr-R) > R*1e-4 + 1e-6) is_sphere=false;
                }
                if (is_sphere) {
                    Vector dm(Pm[0]-C[0],Pm[1]-C[1],Pm[2]-C[2]);
                    double dz=dm[0]*Zs[0]+dm[1]*Zs[1]+dm[2]*Zs[2];
                    Vector Xs(dm[0]-dz*Zs[0],dm[1]-dz*Zs[1],dm[2]-dz*Zs[2]);
                    double xn=Xs.magnitude(); Xs=Vector(Xs[0]/xn,Xs[1]/xn,Xs[2]/xn);
                    Vector Ys=cross(Zs,Xs);
                    auto dot3=[](const Vector&a,const Vector&b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];};
                    have_analytic=true;
                    for (int li : face.loop_indices) {
                        if (li<0||li>=(int)m_loops.size()) continue;
                        // chained, ordered 3D polyline of the loop (greedy walk like loop_vector_area)
                        struct Seg{const NurbsCurve* pc;double t0,t1;Point ps,pe;};
                        std::vector<Seg> segs;
                        for(int ti:m_loops[li].trim_indices){ if(ti<0||ti>=(int)m_trims.size())continue;
                            int c2=m_trims[ti].curve_2d_index; if(c2<0||c2>=(int)m_curves_2d.size())continue;
                            const NurbsCurve&pc=m_curves_2d[c2];auto dc=pc.domain();
                            segs.push_back({&pc,dc.first,dc.second,pc.point_at(dc.first),pc.point_at(dc.second)});}
                        if(segs.empty())continue;
                        std::vector<std::pair<int,bool>> order; std::vector<bool> used(segs.size(),false);
                        order.push_back({0,true}); used[0]=true; Point tail=segs[0].pe;
                        auto d2=[](const Point&a,const Point&b){double dx=a[0]-b[0],dy=a[1]-b[1];return dx*dx+dy*dy;};
                        for(size_t k=1;k<segs.size();++k){int best=-1;bool fwd=true;double bd=1e300;
                            for(size_t j=0;j<segs.size();++j){if(used[j])continue;
                                double ds=d2(segs[j].ps,tail),de=d2(segs[j].pe,tail);
                                if(ds<bd){bd=ds;best=(int)j;fwd=true;} if(de<bd){bd=de;best=(int)j;fwd=false;}}
                            if(best<0)break; used[best]=true; order.push_back({best,fwd});
                            tail=fwd?segs[best].pe:segs[best].ps;}
                        // sample ordered UV polyline (dense -> boundary integral converges to the
                        // pcurve's enclosed flux; residual is the pullback pcurve's own CV density).
                        std::vector<std::array<double,2>> uvpts;
                        for(auto&od:order){const Seg&sg=segs[od.first];int n=200;
                            for(int s=0;s<n;++s){double tt=od.second?sg.t0+(sg.t1-sg.t0)*s/n:sg.t1-(sg.t1-sg.t0)*s/n;
                                Point uv=sg.pc->point_at(tt); uvpts.push_back({uv[0],uv[1]});}}
                        if(uvpts.size()<3)continue;
                        // close it
                        uvpts.push_back(uvpts.front());
                        double uvArea=0.0; for(size_t i=0;i+1<uvpts.size();++i)
                            uvArea+=uvpts[i][0]*uvpts[i+1][1]-uvpts[i+1][0]*uvpts[i][1]; uvArea*=0.5;
                        // 3D points + integrals
                        Vector Acc(0,0,0); double H=0.0; double prevTheta=0.0; Point prevP(0,0,0); double prevH=0.0; double wind=0.0;
                        for(size_t i=0;i<uvpts.size();++i){
                            Point P=srf.point_at(uvpts[i][0],uvpts[i][1]);
                            Vector d(P[0]-C[0],P[1]-C[1],P[2]-C[2]);
                            double theta=std::atan2(dot3(d,Ys),dot3(d,Xs));
                            double h=dot3(d,Zs);
                            if(i>0){
                                // vector area: 0.5 * sum P_i x P_{i+1}
                                Acc=Vector(Acc[0]+prevP[1]*P[2]-prevP[2]*P[1],
                                           Acc[1]+prevP[2]*P[0]-prevP[0]*P[2],
                                           Acc[2]+prevP[0]*P[1]-prevP[1]*P[0]);
                                const double PI=3.14159265358979323846;
                                double dth=theta-prevTheta;
                                while(dth> PI)dth-=2*PI; while(dth<-PI)dth+=2*PI;
                                H += 0.5*(h+prevH)*dth; wind += dth;
                            }
                            prevP=P; prevTheta=theta; prevH=h;
                        }
                        // A loop netting a full +/-2*pi turn in longitude ENCIRCLES a sphere pole;
                        // the Green's reduction (flux = C.A - R^2 * closed-integral h dtheta) then
                        // gains a spurious 2*pi*R^2 pole residue and is invalid for this face.
                        if (std::abs(wind) > 3.14159265358979323846) pole_face = true;
                        Vector A(0.5*Acc[0],0.5*Acc[1],0.5*Acc[2]);
                        double loopflux = (C[0]*A[0]+C[1]*A[1]+C[2]*A[2]) - R*R*H;
                        double osign = (m_loops[li].type==BRepLoopType::Outer?1.0:-1.0) * (uvArea>=0?1.0:-1.0);
                        flux_analytic += osign*loopflux;
                    }
                }
            }
        }
        // For a sphere cap-cut face (non-rectangular trim) the analytic boundary integral is exact;
        // the masked Gauss has a ~1% region error there. Keep Gauss for rectangular trims (full
        // sphere / equatorial band, where it is already exact) and for non-sphere curved faces.
        if (have_analytic && !curved_rect && !pole_face) flux_nat = flux_analytic;
        total += sign * flux_nat;
    }
    return std::abs(total) / 3.0;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Building
///////////////////////////////////////////////////////////////////////////////////////////

int BRep::add_surface(const NurbsSurface& srf) {
    m_surfaces.push_back(srf);
    return (int)m_surfaces.size() - 1;
}

int BRep::add_curve_3d(const NurbsCurve& crv) {
    m_curves_3d.push_back(crv);
    return (int)m_curves_3d.size() - 1;
}

int BRep::add_curve_2d(const NurbsCurve& crv) {
    m_curves_2d.push_back(crv);
    return (int)m_curves_2d.size() - 1;
}

int BRep::add_vertex(const Point& pt) {
    m_vertices.push_back(pt);
    return (int)m_vertices.size() - 1;
}

int BRep::add_edge(int curve_3d_idx, int start_vertex, int end_vertex) {
    BRepEdge e;
    e.curve_3d_index = curve_3d_idx;
    e.start_vertex = start_vertex;
    e.end_vertex = end_vertex;
    m_topology_edges.push_back(e);
    return (int)m_topology_edges.size() - 1;
}

int BRep::add_trim(int curve_2d_idx, int edge_idx, int loop_idx, bool reversed, BRepTrimType type) {
    BRepTrim t;
    t.curve_2d_index = curve_2d_idx;
    t.edge_index = edge_idx;
    t.loop_index = loop_idx;
    t.reversed = reversed;
    t.type = type;
    int idx = (int)m_trims.size();
    m_trims.push_back(t);
    if (loop_idx >= 0 && loop_idx < (int)m_loops.size())
        m_loops[loop_idx].trim_indices.push_back(idx);
    if (edge_idx >= 0 && edge_idx < (int)m_topology_edges.size())
        m_topology_edges[edge_idx].trim_indices.push_back(idx);
    return idx;
}

int BRep::add_loop(int face_idx, BRepLoopType type) {
    BRepLoop l;
    l.face_index = face_idx;
    l.type = type;
    int idx = (int)m_loops.size();
    m_loops.push_back(l);
    if (face_idx >= 0 && face_idx < (int)m_faces.size())
        m_faces[face_idx].loop_indices.push_back(idx);
    return idx;
}

int BRep::add_face(int surface_idx, bool reversed) {
    BRepFace f;
    f.surface_index = surface_idx;
    f.reversed = reversed;
    m_faces.push_back(f);
    return (int)m_faces.size() - 1;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Splitting
///////////////////////////////////////////////////////////////////////////////////////////

namespace {

inline long long q6(double x) { return (long long)std::llround(x * 1000000.0); }

std::pair<std::array<double, 3>, std::array<double, 3>> aabb_from_surface(const NurbsSurface& srf) {
    const int n = 6;
    auto du = srf.domain(0);
    auto dv = srf.domain(1);
    std::array<double, 3> lo = {1e30, 1e30, 1e30};
    std::array<double, 3> hi = {-1e30, -1e30, -1e30};
    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= n; ++j) {
            double u = du.first + (du.second - du.first) * i / n;
            double v = dv.first + (dv.second - dv.first) * j / n;
            Point p = srf.point_at(u, v);
            for (int k = 0; k < 3; ++k) {
                if (p[k] < lo[k]) lo[k] = p[k];
                if (p[k] > hi[k]) hi[k] = p[k];
            }
        }
    }
    return {lo, hi};
}

std::pair<std::array<double, 3>, std::array<double, 3>> aabb_from_curve(const NurbsCurve& crv) {
    const int n = 16;
    auto dc = crv.domain();
    std::array<double, 3> lo = {1e30, 1e30, 1e30};
    std::array<double, 3> hi = {-1e30, -1e30, -1e30};
    for (int i = 0; i <= n; ++i) {
        Point p = crv.point_at(dc.first + (dc.second - dc.first) * i / n);
        for (int k = 0; k < 3; ++k) {
            if (p[k] < lo[k]) lo[k] = p[k];
            if (p[k] > hi[k]) hi[k] = p[k];
        }
    }
    return {lo, hi};
}

bool aabb_overlap(const std::pair<std::array<double, 3>, std::array<double, 3>>& a,
                  const std::pair<std::array<double, 3>, std::array<double, 3>>& b, double m) {
    for (int k = 0; k < 3; ++k) {
        if (a.first[k] - m > b.second[k] || b.first[k] - m > a.second[k]) return false;
    }
    return true;
}

}  // namespace

BRep BRep::split_by_plane(const Plane& plane, double tolerance) const {
    return split_with(tolerance, [&](const NurbsSurface& srf) {
        std::vector<NurbsCurve> out;
        for (auto& pr : Intersection::surface_plane_uv(srf, plane, tolerance)) out.push_back(pr.second);
        return out;
    });
}

BRep BRep::split_by_surface(const NurbsSurface& cutter, double tolerance) const {
    auto cutter_bb = aabb_from_surface(cutter);
    return split_with(tolerance, [&](const NurbsSurface& srf) {
        std::vector<NurbsCurve> out;
        auto srf_bb = aabb_from_surface(srf);
        double margin = std::max({srf_bb.second[0] - srf_bb.first[0],
                                  srf_bb.second[1] - srf_bb.first[1],
                                  srf_bb.second[2] - srf_bb.first[2]}) * 1e-3;
        if (!aabb_overlap(srf_bb, cutter_bb, margin)) return out;
        for (auto& tr : Intersection::surface_surface(srf, cutter, tolerance)) out.push_back(std::get<1>(tr));
        return out;
    });
}

BRep BRep::split_by_curves(const std::vector<NurbsCurve>& curves, double tolerance) const {
    std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>> curve_bbs;
    for (auto& c : curves) curve_bbs.push_back(aabb_from_curve(c));
    return split_with(tolerance, [&](const NurbsSurface& srf) {
        std::vector<NurbsCurve> out;
        auto srf_bb = aabb_from_surface(srf);
        double margin = std::max({srf_bb.second[0] - srf_bb.first[0],
                                  srf_bb.second[1] - srf_bb.first[1],
                                  srf_bb.second[2] - srf_bb.first[2]}) * 1e-3;
        for (size_t ci = 0; ci < curves.size(); ++ci) {
            if (!aabb_overlap(srf_bb, curve_bbs[ci], margin)) continue;
            for (auto& pc : Closest::surface_curve(srf, curves[ci], 0.0, 0.0, tolerance)) out.push_back(pc);
        }
        return out;
    });
}

BRep BRep::split_by_line(const Line& line, double tolerance) const {
    std::vector<Point> pts = {line.start(), line.end()};
    NurbsCurve crv = NurbsCurve::create(false, 1, pts);
    return split_by_curves({crv}, tolerance);
}

BRep BRep::subset(const std::vector<int>& face_indices) const {
    BRep sub;
    sub.name = name;
    std::map<int, int> s_map, c2_map, c3_map, v_map, e_map;

    auto map_surface = [&](int i) -> int {
        auto it = s_map.find(i);
        if (it != s_map.end()) return it->second;
        int x = sub.add_surface(m_surfaces[i]);
        s_map[i] = x;
        return x;
    };
    auto map_c2 = [&](int i) -> int {
        if (i < 0 || i >= (int)m_curves_2d.size()) return -1;
        auto it = c2_map.find(i);
        if (it != c2_map.end()) return it->second;
        int x = sub.add_curve_2d(m_curves_2d[i]);
        c2_map[i] = x;
        return x;
    };
    auto map_vertex = [&](int i) -> int {
        if (i < 0 || i >= (int)m_topology_vertices.size()) return -1;
        auto it = v_map.find(i);
        if (it != v_map.end()) return it->second;
        int idx = sub.add_vertex(m_vertices[m_topology_vertices[i].point_index]);
        BRepVertex tv;
        tv.point_index = idx;
        sub.m_topology_vertices.push_back(tv);
        int nv = (int)sub.m_topology_vertices.size() - 1;
        v_map[i] = nv;
        return nv;
    };
    auto map_edge = [&](int i) -> int {
        if (i < 0 || i >= (int)m_topology_edges.size()) return -1;
        auto it = e_map.find(i);
        if (it != e_map.end()) return it->second;
        const BRepEdge& e = m_topology_edges[i];
        int ci3 = -1;
        if (e.curve_3d_index >= 0 && e.curve_3d_index < (int)m_curves_3d.size()) {
            auto c3 = c3_map.find(e.curve_3d_index);
            if (c3 != c3_map.end()) ci3 = c3->second;
            else { ci3 = sub.add_curve_3d(m_curves_3d[e.curve_3d_index]); c3_map[e.curve_3d_index] = ci3; }
        }
        int sv = map_vertex(e.start_vertex);
        int ev = map_vertex(e.end_vertex);
        int ne = sub.add_edge(ci3, sv, ev);
        e_map[i] = ne;
        return ne;
    };

    for (int fi : face_indices) {
        const BRepFace& face = m_faces[fi];
        int si = map_surface(face.surface_index);
        int new_fi = sub.add_face(si, face.reversed);
        for (int li : face.loop_indices) {
            const BRepLoop& lp = m_loops[li];
            int new_li = sub.add_loop(new_fi, lp.type);
            for (int ti : lp.trim_indices) {
                const BRepTrim& trim = m_trims[ti];
                int ci2 = map_c2(trim.curve_2d_index);
                int ei = map_edge(trim.edge_index);
                sub.add_trim(ci2, ei, new_li, trim.reversed, trim.type);
            }
        }
    }
    for (int ei = 0; ei < (int)sub.m_topology_edges.size(); ++ei) {
        int sv = sub.m_topology_edges[ei].start_vertex;
        int ev = sub.m_topology_edges[ei].end_vertex;
        if (sv >= 0 && sv < (int)sub.m_topology_vertices.size())
            sub.m_topology_vertices[sv].edge_indices.push_back(ei);
        if (ev != sv && ev >= 0 && ev < (int)sub.m_topology_vertices.size())
            sub.m_topology_vertices[ev].edge_indices.push_back(ei);
    }
    return sub;
}

std::vector<BRep> BRep::split_by_plane_pieces(const Plane& plane, double tolerance) const {
    BRep whole = split_by_plane(plane, tolerance);
    const Point& o = plane.origin();
    const Vector& n = plane.z_axis();
    std::vector<int> pos, neg;
    for (int fi = 0; fi < (int)whole.m_faces.size(); ++fi) {
        const BRepFace& face = whole.m_faces[fi];
        const NurbsSurface& srf = whole.m_surfaces[face.surface_index];
        double sx = 0.0, sy = 0.0, sz = 0.0;
        int cnt = 0;
        for (int li : face.loop_indices) {
            const BRepLoop& lp = whole.m_loops[li];
            if (lp.type != BRepLoopType::Outer) continue;
            for (int ti : lp.trim_indices) {
                const NurbsCurve& pc = whole.m_curves_2d[whole.m_trims[ti].curve_2d_index];
                auto dc = pc.domain();
                for (int k = 0; k < 8; ++k) {
                    Point uv = pc.point_at(dc.first + (dc.second - dc.first) * k / 8.0);
                    Point p = srf.point_at(uv[0], uv[1]);
                    sx += p[0]; sy += p[1]; sz += p[2]; cnt += 1;
                }
            }
        }
        if (cnt == 0) continue;
        double cx = sx / cnt, cy = sy / cnt, cz = sz / cnt;
        double d = (cx - o[0]) * n[0] + (cy - o[1]) * n[1] + (cz - o[2]) * n[2];
        if (d >= 0.0) pos.push_back(fi); else neg.push_back(fi);
    }
    std::vector<BRep> pieces;
    if (!pos.empty()) pieces.push_back(whole.subset(pos));
    if (!neg.empty()) pieces.push_back(whole.subset(neg));
    return pieces;
}

BRep BRep::split_by_brep(const BRep& cutter, double tolerance) const {
    std::vector<std::pair<std::array<double, 3>, std::array<double, 3>>> cutter_bbs;
    for (const auto& cs : cutter.m_surfaces) cutter_bbs.push_back(aabb_from_surface(cs));
    return split_with(tolerance, [&](const NurbsSurface& srf) {
        std::vector<NurbsCurve> out;
        auto srf_bb = aabb_from_surface(srf);
        double margin = std::max({srf_bb.second[0] - srf_bb.first[0],
                                  srf_bb.second[1] - srf_bb.first[1],
                                  srf_bb.second[2] - srf_bb.first[2]}) * 1e-3;
        for (size_t ci = 0; ci < cutter.m_surfaces.size(); ++ci) {
            if (!aabb_overlap(srf_bb, cutter_bbs[ci], margin)) continue;
            for (auto& pc : Intersection::cut_curves_on_surface(srf, cutter.m_surfaces[ci], tolerance)) out.push_back(pc);
        }
        return out;
    });
}

BRep BRep::split_with(double tolerance, const std::function<std::vector<NurbsCurve>(const NurbsSurface&)>& cut_for) const {
    BRep result;
    result.name = name;
    std::map<std::tuple<long long, long long, long long>, int> vmap;
    std::map<std::tuple<int, int, long long, long long, long long>, int> emap;
    static const bool s_prof = (std::getenv("SESSION_BOOL_PROFILE") != nullptr);
    double prof_ssi = 0, prof_arr = 0, prof_lift = 0;
    auto pf_now = []{ return std::chrono::high_resolution_clock::now(); };
    auto pf_us = [](auto a, auto b){ return std::chrono::duration<double, std::micro>(b - a).count(); };

    auto lift_loop = [&](const NurbsSurface& srf, double devtol, const NurbsCurve& pc,
                         NurbsCurve& c3d, Point& p0, Point& p1, Point& pm) {
        // Lift a 2D pcurve onto the surface as a 3D polyline edge, refining ADAPTIVELY by 3D
        // chord deviation: a straight lift (box segment on a planar face) terminates at 2 points,
        // while a 2-CV UV line that wraps a cylinder into a full circle is subdivided to the chord
        // tolerance. This is the boolean's hot path -- a fixed dense sample count made point_at
        // dominate the whole operation.
        auto dc = pc.domain();
        auto eval = [&](double t) -> Point { Point uv = pc.point_at(t); return srf.point_at(uv[0], uv[1]); };
        std::function<void(double, const Point&, double, const Point&, int, std::vector<Point>&)> rec =
            [&](double ta, const Point& pa, double tb, const Point& pb, int depth, std::vector<Point>& acc) {
                double tm = 0.5 * (ta + tb); Point pmid = eval(tm);
                double ex = pb[0]-pa[0], ey = pb[1]-pa[1], ez = pb[2]-pa[2];
                double L2 = ex*ex + ey*ey + ez*ez, dev;
                if (L2 > 1e-30) {
                    double t = ((pmid[0]-pa[0])*ex+(pmid[1]-pa[1])*ey+(pmid[2]-pa[2])*ez)/L2;
                    double cx = pa[0]+t*ex, cy = pa[1]+t*ey, cz = pa[2]+t*ez;
                    dev = std::sqrt((pmid[0]-cx)*(pmid[0]-cx)+(pmid[1]-cy)*(pmid[1]-cy)+(pmid[2]-cz)*(pmid[2]-cz));
                } else {
                    dev = std::sqrt((pmid[0]-pa[0])*(pmid[0]-pa[0])+(pmid[1]-pa[1])*(pmid[1]-pa[1])+(pmid[2]-pa[2])*(pmid[2]-pa[2]));
                }
                if (dev > devtol && depth < 9) {
                    rec(ta, pa, tm, pmid, depth+1, acc);
                    acc.push_back(pmid);
                    rec(tm, pmid, tb, pb, depth+1, acc);
                }
            };
        int n0 = std::max(pc.cv_count() - 1, 1);
        std::vector<Point> pts3;
        Point prev = eval(dc.first);
        pts3.push_back(prev);
        for (int i = 0; i < n0; ++i) {
            double ta = dc.first + (dc.second - dc.first) * i / n0;
            double tb = dc.first + (dc.second - dc.first) * (i + 1) / n0;
            Point pa = (i == 0) ? prev : eval(ta);
            Point pb = eval(tb);
            rec(ta, pa, tb, pb, 0, pts3);
            pts3.push_back(pb);
        }
        c3d = NurbsCurve::create(false, 1, pts3);
        p0 = pts3.front();
        p1 = pts3.back();
        pm = pts3[pts3.size() / 2];
    };

    auto find_or_add_vertex = [&](const Point& p) -> int {
        auto key = std::make_tuple(q6(p[0]), q6(p[1]), q6(p[2]));
        auto it = vmap.find(key);
        if (it != vmap.end()) return it->second;
        int idx = result.add_vertex(p);
        BRepVertex tv;
        tv.point_index = idx;
        result.m_topology_vertices.push_back(tv);
        vmap[key] = idx;
        return idx;
    };

    auto append_face = [&](const NurbsSurface& srf,
                           const std::vector<std::pair<BRepLoopType, std::vector<NurbsCurve>>>& loops) {
        int si = result.add_surface(srf);
        int fi = result.add_face(si, false);
        // One deviation tolerance per face (chord error target = 5e-4 of the surface size): a
        // straight lift stays at 2 points, a wrapped circle subdivides to within the sew tol.
        double sd; {
            double bmn[3]={1e30,1e30,1e30}, bmx[3]={-1e30,-1e30,-1e30};
            for (int i = 0; i < srf.cv_count(0); ++i) for (int j = 0; j < srf.cv_count(1); ++j) {
                Point p = srf.get_cv(i, j);
                for (int k = 0; k < 3; ++k) { bmn[k]=std::min(bmn[k],p[k]); bmx[k]=std::max(bmx[k],p[k]); }
            }
            sd = std::sqrt((bmx[0]-bmn[0])*(bmx[0]-bmn[0])+(bmx[1]-bmn[1])*(bmx[1]-bmn[1])+(bmx[2]-bmn[2])*(bmx[2]-bmn[2]));
            if (sd < 1e-12) sd = 1.0;
        }
        double devtol = sd * 2e-3;
        for (const auto& lp : loops) {
            int li = result.add_loop(fi, lp.first);
            for (const auto& pc : lp.second) {
                if (!pc.is_valid()) continue;
                NurbsCurve c3d;
                Point p0, p1, pm;
                lift_loop(srf, devtol, pc, c3d, p0, p1, pm);
                int ci3d = result.add_curve_3d(c3d);
                int va = find_or_add_vertex(p0);
                int vb = find_or_add_vertex(p1);
                int lo = std::min(va, vb), hi = std::max(va, vb);
                auto ekey = std::make_tuple(lo, hi, q6(pm[0]), q6(pm[1]), q6(pm[2]));
                int ei;
                BRepTrimType ttype;
                auto it = emap.find(ekey);
                if (it != emap.end()) {
                    ei = it->second;
                    ttype = BRepTrimType::Mated;
                } else {
                    ei = result.add_edge(ci3d, lo, hi);
                    emap[ekey] = ei;
                    ttype = BRepTrimType::Boundary;
                }
                int ci2d = result.add_curve_2d(pc);
                result.add_trim(ci2d, ei, li, false, ttype);
            }
        }
    };

    for (const auto& face : m_faces) {
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        std::vector<NurbsCurve> outer_pcs;
        std::vector<std::vector<NurbsCurve>> inner_loops;
        bool has_inner = false;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const BRepLoop& bloop = m_loops[li];
            std::vector<NurbsCurve> pcs;
            for (int ti : bloop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int c2 = m_trims[ti].curve_2d_index;
                if (c2 >= 0 && c2 < (int)m_curves_2d.size()) pcs.push_back(m_curves_2d[c2]);
            }
            if (bloop.type == BRepLoopType::Inner) {
                has_inner = true;
                inner_loops.push_back(pcs);
            } else {
                outer_pcs = pcs;
            }
        }

        auto pf_t0 = pf_now();
        std::vector<NurbsCurve> cut_pcs = cut_for(srf);
        if (s_prof) prof_ssi += pf_us(pf_t0, pf_now());
        if (cut_pcs.empty() || has_inner) {
            std::vector<std::pair<BRepLoopType, std::vector<NurbsCurve>>> loops;
            loops.push_back({BRepLoopType::Outer, outer_pcs});
            for (auto& il : inner_loops) loops.push_back({BRepLoopType::Inner, il});
            auto pf_t1 = pf_now();
            append_face(srf, loops);
            if (s_prof) prof_lift += pf_us(pf_t1, pf_now());
            continue;
        }

        int n_boundary = (int)outer_pcs.size();
        std::vector<NurbsCurve> all_pcs = outer_pcs;
        all_pcs.insert(all_pcs.end(), cut_pcs.begin(), cut_pcs.end());
        auto pf_t2 = pf_now();
        // Seam-aware WireSplitter arrangement is opt-in via SESSION_WIRESPLIT; with the flag unset
        // the kernel is byte-identical to the proven split_by_uv_curves path.
        static const bool s_wiresplit = (std::getenv("SESSION_WIRESPLIT") != nullptr);
        std::vector<NurbsSurfaceTrimmed> parts = s_wiresplit
            ? NurbsSurfaceTrimmed::split_face_by_wires(srf, cut_pcs, outer_pcs, tolerance)
            : NurbsSurfaceTrimmed::split_by_uv_curves(srf, all_pcs, tolerance, false, n_boundary);
        if (s_prof) prof_arr += pf_us(pf_t2, pf_now());
        if (parts.size() <= 1) {
            std::vector<std::pair<BRepLoopType, std::vector<NurbsCurve>>> loops;
            loops.push_back({BRepLoopType::Outer, outer_pcs});
            auto pf_t3 = pf_now();
            append_face(srf, loops);
            if (s_prof) prof_lift += pf_us(pf_t3, pf_now());
            continue;
        }
        auto pf_t4 = pf_now();
        for (const auto& part : parts) {
            // Prefer the per-run segmentation (each boundary run a separate pcurve) so each
            // run lifts to its own edge and mates with the matching segment edge of an
            // adjacent face -> watertight imprint. Fall back to the single joined loop.
            std::vector<std::pair<BRepLoopType, std::vector<NurbsCurve>>> loops;
            loops.push_back({BRepLoopType::Outer,
                part.m_outer_segments.empty() ? std::vector<NurbsCurve>{part.m_outer_loop}
                                              : part.m_outer_segments});
            for (size_t k = 0; k < part.m_inner_loops.size(); ++k) {
                const std::vector<NurbsCurve>& isegs =
                    (k < part.m_inner_segments.size()) ? part.m_inner_segments[k]
                                                       : std::vector<NurbsCurve>{};
                loops.push_back({BRepLoopType::Inner,
                    isegs.empty() ? std::vector<NurbsCurve>{part.m_inner_loops[k]} : isegs});
            }
            append_face(part.m_surface, loops);
        }
        if (s_prof) prof_lift += pf_us(pf_t4, pf_now());
    }
    if (s_prof) std::fprintf(stderr, "[split-prof]     ssi=%.1f arr=%.1f lift=%.1f us\n", prof_ssi, prof_arr, prof_lift);

    for (int ei = 0; ei < (int)result.m_topology_edges.size(); ++ei) {
        int sv = result.m_topology_edges[ei].start_vertex;
        int ev = result.m_topology_edges[ei].end_vertex;
        if (sv >= 0 && sv < (int)result.m_topology_vertices.size())
            result.m_topology_vertices[sv].edge_indices.push_back(ei);
        if (ev != sv && ev >= 0 && ev < (int)result.m_topology_vertices.size())
            result.m_topology_vertices[ev].edge_indices.push_back(ei);
    }
    return result;
}

void BRep::imprint_edges(double tol) {
    double diag;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& p : m_vertices) {
            xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
            xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        }
        diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (diag <= 0) diag = 1.0;
    }
    if (tol <= 0.0) tol = diag * 1e-6;

    // Candidate split positions: every topology-vertex position.
    std::vector<Point> vpos;
    vpos.reserve(m_topology_vertices.size());
    for (const auto& tv : m_topology_vertices)
        vpos.push_back((tv.point_index >= 0 && tv.point_index < (int)m_vertices.size())
                       ? m_vertices[tv.point_index] : Point(1e300, 1e300, 1e300));

    // Split a curve at a set of (sorted, strictly-interior) parameters into consecutive pieces.
    auto split_multi = [](const NurbsCurve& c, std::vector<double> params) -> std::vector<NurbsCurve> {
        std::sort(params.begin(), params.end());
        std::vector<NurbsCurve> out;
        NurbsCurve rem = c;
        for (double t : params) {
            auto [l, r] = rem.split(t);
            if (!l.is_valid() || !r.is_valid()) return {};  // fail -> caller keeps original
            out.push_back(l);
            rem = r;
        }
        out.push_back(rem);
        return out;
    };

    int ne0 = (int)m_topology_edges.size();
    for (int ei = 0; ei < ne0; ++ei) {
        // Only repair under-mated edges (the ones that break watertightness).
        if ((int)m_topology_edges[ei].trim_indices.size() >= 2) continue;
        int ci = m_topology_edges[ei].curve_3d_index;
        if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
        NurbsCurve C = m_curves_3d[ci];
        if (!C.is_valid()) continue;
        Point pA = C.point_at_start(), pB = C.point_at_end();
        double clen = pA.distance(pB);
        if (clen < tol) continue;  // closed cut circles: co-split needs the seam-aware arrangement

        // Edge AABB (sampled) -> cheap reject of far vertices before the closest_parameter solve.
        auto cdomf = C.domain();
        double ebb[6] = {1e300,1e300,1e300,-1e300,-1e300,-1e300};
        for (int k = 0; k <= 6; ++k) {
            Point p = C.point_at(cdomf.first + (cdomf.second-cdomf.first)*k/6);
            for (int d = 0; d < 3; ++d) { ebb[d]=std::min(ebb[d],p[d]); ebb[d+3]=std::max(ebb[d+3],p[d]); }
        }

        // Interior vertices that lie on C (a T-junction split point).
        struct Split { double tc; Point V; };
        std::vector<Split> splits;
        for (const Point& V : vpos) {
            if (V[0] > 1e299) continue;
            if (V[0] < ebb[0]-tol || V[0] > ebb[3]+tol || V[1] < ebb[1]-tol || V[1] > ebb[4]+tol
                || V[2] < ebb[2]-tol || V[2] > ebb[5]+tol) continue;
            if (V.distance(pA) < tol || V.distance(pB) < tol) continue;
            double tc = C.closest_parameter(V);
            if (C.point_at(tc).distance(V) > tol) continue;
            auto dom = C.domain();
            double frac = (tc - dom.first) / (dom.second - dom.first);
            if (frac <= 1e-6 || frac >= 1.0 - 1e-6) continue;
            // Dedup near-equal split points.
            bool dup = false;
            for (auto& s : splits) if (s.V.distance(V) < tol) { dup = true; break; }
            if (!dup) splits.push_back({tc, V});
        }
        if (splits.empty()) continue;
        std::sort(splits.begin(), splits.end(), [](const Split& a, const Split& b){ return a.tc < b.tc; });

        std::vector<double> cparams;
        for (auto& s : splits) cparams.push_back(s.tc);
        std::vector<NurbsCurve> c3pieces = split_multi(C, cparams);
        if ((int)c3pieces.size() != (int)splits.size() + 1) continue;  // split failed

        // Snapshot the original trims BEFORE the edge-piece loop clears edge ei's trim list.
        std::vector<int> orig_trims = m_topology_edges[ei].trim_indices;

        // New topology vertices at the split points.
        std::vector<int> vids;
        vids.push_back(m_topology_edges[ei].start_vertex);
        for (auto& s : splits) {
            int pidx = add_vertex(s.V);
            BRepVertex tv; tv.point_index = pidx;
            int tvid = (int)m_topology_vertices.size();
            m_topology_vertices.push_back(tv);
            vids.push_back(tvid);
        }
        vids.push_back(m_topology_edges[ei].end_vertex);

        // Edge pieces: piece 0 reuses ei (+ its 3D curve slot), the rest are appended.
        std::vector<int> edge_ids;
        for (size_t j = 0; j < c3pieces.size(); ++j) {
            int c3idx, eidx;
            if (j == 0) { c3idx = ci; eidx = ei; m_curves_3d[ci] = c3pieces[0]; }
            else { c3idx = add_curve_3d(c3pieces[j]); eidx = (int)m_topology_edges.size();
                   m_topology_edges.push_back(BRepEdge()); }
            m_topology_edges[eidx].curve_3d_index = c3idx;
            m_topology_edges[eidx].start_vertex = vids[j];
            m_topology_edges[eidx].end_vertex = vids[j + 1];
            m_topology_edges[eidx].trim_indices.clear();
            edge_ids.push_back(eidx);
        }

        // Split each trim of the (formerly single) edge to match the edge pieces.
        for (int ti : orig_trims) {
            if (ti < 0 || ti >= (int)m_trims.size()) continue;
            BRepTrim T = m_trims[ti];
            int li = T.loop_index;
            int fi = (li >= 0 && li < (int)m_loops.size()) ? m_loops[li].face_index : -1;
            int si = (fi >= 0 && fi < (int)m_faces.size()) ? m_faces[fi].surface_index : -1;
            int c2 = T.curve_2d_index;
            std::vector<NurbsCurve> p2pieces;
            if (si >= 0 && si < (int)m_surfaces.size() && c2 >= 0 && c2 < (int)m_curves_2d.size()) {
                const NurbsSurface& S = m_surfaces[si];
                NurbsCurve P = m_curves_2d[c2];
                std::vector<double> tps;
                for (auto& s : splits) {
                    auto [uu, vv, dd] = Closest::surface_point(S, s.V);
                    double tp = P.closest_parameter(Point(uu, vv, 0.0));
                    tps.push_back(tp);
                }
                p2pieces = split_multi(P, tps);
            }
            bool ok = ((int)p2pieces.size() == (int)edge_ids.size());
            // Build the per-piece trims (forward order; boolean trims are not reversed).
            std::vector<int> newtrims;
            for (size_t j = 0; j < edge_ids.size(); ++j) {
                int c2idx, tidx;
                NurbsCurve pc = ok ? p2pieces[j] : (j == 0 ? m_curves_2d[std::max(c2,0)] : NurbsCurve());
                if (j == 0) { c2idx = (c2 >= 0 ? c2 : add_curve_2d(pc)); if (c2 >= 0) m_curves_2d[c2] = pc; tidx = ti; }
                else { c2idx = add_curve_2d(pc); tidx = (int)m_trims.size(); m_trims.push_back(BRepTrim()); }
                m_trims[tidx].curve_2d_index = c2idx;
                m_trims[tidx].edge_index = edge_ids[j];
                m_trims[tidx].loop_index = li;
                m_trims[tidx].reversed = T.reversed;
                m_trims[tidx].type = T.type;
                newtrims.push_back(tidx);
                m_topology_edges[edge_ids[j]].trim_indices.push_back(tidx);
            }
            if (T.reversed) std::reverse(newtrims.begin(), newtrims.end());
            // Replace ti in the loop's trim list with the piece trims.
            if (li >= 0 && li < (int)m_loops.size()) {
                auto& tl = m_loops[li].trim_indices;
                auto it = std::find(tl.begin(), tl.end(), ti);
                if (it != tl.end()) { int pos = (int)(it - tl.begin());
                    tl.erase(it); tl.insert(tl.begin() + pos, newtrims.begin(), newtrims.end()); }
            }
        }
    }

    // Rebuild vertex->edge adjacency.
    for (auto& v : m_topology_vertices) v.edge_indices.clear();
    for (int ei = 0; ei < (int)m_topology_edges.size(); ++ei) {
        int sv = m_topology_edges[ei].start_vertex, ev = m_topology_edges[ei].end_vertex;
        if (sv >= 0 && sv < (int)m_topology_vertices.size()) m_topology_vertices[sv].edge_indices.push_back(ei);
        if (ev != sv && ev >= 0 && ev < (int)m_topology_vertices.size()) m_topology_vertices[ev].edge_indices.push_back(ei);
    }
}

void BRep::co_refine_coincident_edges(double tol) {
    double diag;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& p : m_vertices) {
            xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
            xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        }
        diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (diag <= 0) diag = 1.0;
    }
    if (tol <= 0.0) tol = diag * 5e-3;   // sew-level coincidence tolerance

    auto split_multi = [](const NurbsCurve& c, std::vector<double> params) -> std::vector<NurbsCurve> {
        std::sort(params.begin(), params.end());
        std::vector<NurbsCurve> out; NurbsCurve rem = c;
        for (double t : params) { auto [l, r] = rem.split(t);
            if (!l.is_valid() || !r.is_valid()) return {}; out.push_back(l); rem = r; }
        out.push_back(rem); return out;
    };
    auto p2pl = [](const Point& p, const std::vector<Point>& pts) -> double {
        double best = 1e300;
        for (size_t j = 0; j + 1 < pts.size(); ++j) { const Point& a = pts[j]; const Point& b = pts[j+1];
            double ex=b[0]-a[0],ey=b[1]-a[1],ez=b[2]-a[2],L2=ex*ex+ey*ey+ez*ez;
            double t = (L2>1e-30)?((p[0]-a[0])*ex+(p[1]-a[1])*ey+(p[2]-a[2])*ez)/L2:0.0;
            t = std::min(std::max(t,0.0),1.0);
            double cx=a[0]+t*ex,cy=a[1]+t*ey,cz=a[2]+t*ez;
            best = std::min(best, std::sqrt((p[0]-cx)*(p[0]-cx)+(p[1]-cy)*(p[1]-cy)+(p[2]-cz)*(p[2]-cz))); }
        return best;
    };

    int ne0 = (int)m_topology_edges.size();
    auto cand = [&](int e){ return e>=0 && e<ne0 && (int)m_topology_edges[e].trim_indices.size() < 2
                                  && m_topology_edges[e].curve_3d_index >= 0
                                  && m_topology_edges[e].curve_3d_index < (int)m_curves_3d.size(); };
    const int NS = 24;
    std::vector<std::vector<Point>> samp(ne0);
    std::vector<std::array<double,6>> bbox(ne0);
    for (int e=0;e<ne0;++e){ if(!cand(e)) continue;
        const NurbsCurve& C = m_curves_3d[m_topology_edges[e].curve_3d_index];
        auto [t0,t1]=C.domain(); std::array<double,6> bb={1e300,1e300,1e300,-1e300,-1e300,-1e300};
        for(int k=0;k<=NS;++k){ Point p=C.point_at(t0+(t1-t0)*k/NS); samp[e].push_back(p);
            for(int d=0;d<3;++d){bb[d]=std::min(bb[d],p[d]);bb[d+3]=std::max(bb[d+3],p[d]);} }
        bbox[e]=bb; }
    auto bbox_far = [&](int i,int j){ const auto&a=bbox[i];const auto&b=bbox[j];
        return a[0]>b[3]+tol||b[0]>a[3]+tol||a[1]>b[4]+tol||b[1]>a[4]+tol||a[2]>b[5]+tol||b[2]>a[5]+tol; };
    // ej is an arc-SUBSET of ei: every ej sample lies within tol of ei's polyline. (One-directional;
    // a full circle is NOT a subset of one of its arcs, but each arc IS a subset of the circle.)
    auto subset_of = [&](int ej,int ei){ if(samp[ej].size()<2||samp[ei].size()<2) return false;
        for(const auto& p:samp[ej]) if(p2pl(p,samp[ei])>tol) return false; return true; };

    for (int ei=0; ei<ne0; ++ei) {
        if (!cand(ei)) continue;
        const NurbsCurve C = m_curves_3d[m_topology_edges[ei].curve_3d_index];
        if (!C.is_valid()) continue;
        auto dom = C.domain(); Point pA=C.point_at_start(), pB=C.point_at_end();
        bool closed = pA.distance(pB) < tol;

        // Split points on C = endpoints of DISTINCT under-mated edges that are arc-subsets of C and
        // land strictly interior on C (a circle split at its coincident arcs' shared endpoints).
        std::vector<std::pair<double,Point>> sp;
        bool seam_has_split = false;
        for (int ej=0; ej<ne0; ++ej) {
            if (ej==ei || !cand(ej) || bbox_far(ei,ej) || !subset_of(ej,ei)) continue;
            const NurbsCurve& Cj = m_curves_3d[m_topology_edges[ej].curve_3d_index];
            // Only an OPEN arc subdivides ei at its endpoints. A closed circle coincident with ei
            // (e.g. the same cut circle imprinted whole on both operands) has only its arbitrary
            // param-seam as an "endpoint" -- splitting there is spurious; sew merges them whole.
            if (Cj.point_at_start().distance(Cj.point_at_end()) < tol) continue;
            Point ends[2] = { Cj.point_at_start(), Cj.point_at_end() };
            for (const Point& V : ends) {
                double tc = C.closest_parameter(V);
                if (C.point_at(tc).distance(V) > tol) continue;
                double frac = (tc-dom.first)/(dom.second-dom.first);
                if (frac <= 1e-6 || frac >= 1.0-1e-6) { seam_has_split = true; continue; }
                bool dup=false; for(auto&s:sp) if(s.second.distance(V)<tol){dup=true;break;}
                if(!dup) sp.push_back({tc,V});
            }
        }
        if (sp.empty()) continue;
        std::sort(sp.begin(), sp.end(), [](auto&a,auto&b){return a.first<b.first;});
        std::vector<double> iparams; std::vector<Point> ipts;
        for(auto&s:sp){ iparams.push_back(s.first); ipts.push_back(s.second); }

        std::vector<NurbsCurve> c3pieces = split_multi(C, iparams);
        if ((int)c3pieces.size() != (int)iparams.size()+1) continue;  // split failed -> keep edge

        // wrap-join the first+last 3D piece across the param seam ONLY for a closed edge whose seam
        // is interior to an arc (no split point at the seam). Else (open, or seam coincides with a
        // split point) the pieces are already the arcs.
        bool do_wrap = closed && !seam_has_split;

        std::vector<int> orig_trims = m_topology_edges[ei].trim_indices;
        // vertices at the interior split points
        std::vector<int> svids;
        for (const Point& V : ipts) {
            int pidx = add_vertex(V); BRepVertex tv; tv.point_index = pidx;
            svids.push_back((int)m_topology_vertices.size()); m_topology_vertices.push_back(tv);
        }
        // edge-piece curves + their (startV,endV)
        std::vector<NurbsCurve> arcs; std::vector<std::array<int,2>> arc_v;
        if (do_wrap) {
            auto wrap = NurbsCurve::join({c3pieces.back(), c3pieces.front()}, tol);
            if (wrap.size()!=1 || !wrap[0].is_valid()) continue;
            for (size_t k=1;k+1<c3pieces.size();++k){ arcs.push_back(c3pieces[k]); arc_v.push_back({svids[k-1],svids[k]}); }
            arcs.push_back(wrap[0]); arc_v.push_back({svids.back(), svids.front()});
        } else {
            std::vector<int> vids; vids.push_back(m_topology_edges[ei].start_vertex);
            for(int v:svids) vids.push_back(v); vids.push_back(m_topology_edges[ei].end_vertex);
            for (size_t k=0;k<c3pieces.size();++k){ arcs.push_back(c3pieces[k]); arc_v.push_back({vids[k],vids[k+1]}); }
        }

        // create the piece edges (piece 0 reuses ei + its 3D-curve slot)
        std::vector<int> edge_ids;
        for (size_t k=0;k<arcs.size();++k){
            int c3idx,eidx;
            if(k==0){ c3idx=m_topology_edges[ei].curve_3d_index; eidx=ei; m_curves_3d[c3idx]=arcs[0]; }
            else { c3idx=add_curve_3d(arcs[k]); eidx=(int)m_topology_edges.size(); m_topology_edges.push_back(BRepEdge()); }
            m_topology_edges[eidx].curve_3d_index=c3idx;
            m_topology_edges[eidx].start_vertex=arc_v[k][0];
            m_topology_edges[eidx].end_vertex=arc_v[k][1];
            m_topology_edges[eidx].trim_indices.clear();
            edge_ids.push_back(eidx);
        }

        // split each trim's 2D pcurve at the same params (project the split points onto the surface)
        for (int ti : orig_trims) {
            if (ti<0 || ti>=(int)m_trims.size()) continue;
            BRepTrim T = m_trims[ti];
            int li=T.loop_index, fi=(li>=0&&li<(int)m_loops.size())?m_loops[li].face_index:-1;
            int si=(fi>=0&&fi<(int)m_faces.size())?m_faces[fi].surface_index:-1;
            int c2=T.curve_2d_index;
            // Build each arc's 2D pcurve. PLANAR face: project the (already-split) 3D arc onto the
            // plane (exact, cheap). CURVED face: split the original open pcurve at the projected
            // params (cheap, exact) -- projecting onto a curved surface per-sample is slow/unstable
            // near singularities (e.g. a cone apex). (NurbsCurve::split silently fails on a CLOSED
            // rational circle, so closed circles must be on planar faces here -- they are: a closed
            // cut-circle co-refined against open arcs is the box's planar disc; coincident closed
            // circles on curved faces are skipped as ej.)
            std::vector<NurbsCurve> p2;
            if (si>=0 && si<(int)m_surfaces.size()) {
                const NurbsSurface& S=m_surfaces[si];
                if (S.is_planar(nullptr, 1e-6)) {
                    for (const NurbsCurve& arc3 : arcs) {
                        auto ad = arc3.domain(); int n = std::max(arc3.cv_count()*2, 12);
                        std::vector<Point> uvs;
                        for (int s=0;s<=n;++s){ Point P3=arc3.point_at(ad.first+(ad.second-ad.first)*s/n);
                            auto [uu,vv,dd]=Closest::surface_point(S,P3); (void)dd; uvs.push_back(Point(uu,vv,0.0)); }
                        p2.push_back(NurbsCurve::create(false,1,uvs));
                    }
                } else if (c2>=0 && c2<(int)m_curves_2d.size()) {
                    NurbsCurve P=m_curves_2d[c2];
                    std::vector<double> tps;
                    for (const Point& V : ipts){ auto [uu,vv,dd]=Closest::surface_point(S,V); (void)dd;
                        tps.push_back(P.closest_parameter(Point(uu,vv,0.0))); }
                    p2 = split_multi(P, tps);
                    if (do_wrap && (int)p2.size()>=2){ auto w2=NurbsCurve::join({p2.back(),p2.front()}, tol);
                        if(w2.size()==1 && w2[0].is_valid()){ std::vector<NurbsCurve> a2(p2.begin()+1,p2.end()-1);
                            a2.push_back(w2[0]); p2=a2; } else p2.clear(); }
                }
            }
            bool ok = ((int)p2.size()==(int)edge_ids.size());
            std::vector<int> newtrims;
            for (size_t k=0;k<edge_ids.size();++k){
                int c2idx,tidx;
                NurbsCurve pc = ok ? p2[k] : (k==0 ? m_curves_2d[std::max(c2,0)] : NurbsCurve());
                if(k==0){ c2idx=(c2>=0?c2:add_curve_2d(pc)); if(c2>=0)m_curves_2d[c2]=pc; tidx=ti; }
                else { c2idx=add_curve_2d(pc); tidx=(int)m_trims.size(); m_trims.push_back(BRepTrim()); }
                m_trims[tidx].curve_2d_index=c2idx; m_trims[tidx].edge_index=edge_ids[k];
                m_trims[tidx].loop_index=li; m_trims[tidx].reversed=T.reversed; m_trims[tidx].type=T.type;
                newtrims.push_back(tidx); m_topology_edges[edge_ids[k]].trim_indices.push_back(tidx);
            }
            if (T.reversed) std::reverse(newtrims.begin(), newtrims.end());
            if (li>=0 && li<(int)m_loops.size()){ auto& tl=m_loops[li].trim_indices;
                auto it=std::find(tl.begin(),tl.end(),ti);
                if(it!=tl.end()){ int pos=(int)(it-tl.begin()); tl.erase(it); tl.insert(tl.begin()+pos,newtrims.begin(),newtrims.end()); } }
        }
    }
    // rebuild vertex->edge adjacency
    for (auto& v : m_topology_vertices) v.edge_indices.clear();
    for (int ei=0; ei<(int)m_topology_edges.size(); ++ei){ int sv=m_topology_edges[ei].start_vertex, ev=m_topology_edges[ei].end_vertex;
        if(sv>=0&&sv<(int)m_topology_vertices.size()) m_topology_vertices[sv].edge_indices.push_back(ei);
        if(ev!=sv&&ev>=0&&ev<(int)m_topology_vertices.size()) m_topology_vertices[ev].edge_indices.push_back(ei); }
}

void BRep::make_shared_section_edges(const BRep& A, const BRep& B, double tol) {
    // BUILDSPEC P1 — PAVE ENGINE: split-exactly-once + PostTreatFF.
    //
    // *this is the already-combined+imprinted boolean result (A's selected split faces + B's, in
    // one geometry pool). The A&B section curve is computed ONCE per surface pair (the exact
    // analytic conic), then a port of the OCCT pave model decomposes each section curve into its
    // canonical split-exactly-once segment set:
    //   - PutPavesOnCurve     : project existing On/In vertices onto the curve (parametric dedup),
    //   - PutBoundPaveOnCurve  : the two curve ends are paves,
    //   - PutClosingPaveOnCurve: a closed circle gets a closing pave (same vid, opposite bound),
    //   - PostTreatFF          : section/section crossings (3-surface corners) become paves on both,
    //   - update_paveblock     : sort paves by parameter, emit consecutive [t_i,t_{i+1}] segments.
    // Each under-mated section edge is then re-fit to the EXACT sub-arc of that single curve over
    // the pave-segment span it occupies (endpoints SNAPPED to the shared pave parameters), so the
    // box-side and periodic-side arcs of a segment become bit-identical geometry and merge into ONE
    // shared edge (two trims, one per operand) -- watertight by construction. The proven co-refine
    // (topological split of a closed circle vs its arcs) and Hausdorff sew (merge) remain the
    // split/merge backbone; the pave engine supplies the principled, shared split parameters that
    // generalise P0's seam-only paves to arbitrary multi-segment / crossing section topology.

    // ---- OCCT pave model (BOPDS_Pave / BOPDS_PaveBlock) ----
    struct Pave      { int vid; double t; };
    struct PaveBlock { int orig = -1; int edge = -1; Pave p1{-1,0.0}, p2{-1,0.0}; std::vector<Pave> ext; };

    double diag;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& p : m_vertices) {
            xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
            xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        }
        diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (diag <= 0) diag = 1.0;
    }
    if (tol <= 0.0) tol = diag * 5e-3;

    // ---- Phase 1 (PerformFF): the exact section curve(s), computed ONCE per surface pair ----
    struct Sec {
        std::vector<Point> samp; std::array<double,6> bb; NurbsCurve c3d; bool closed;
        double t0, t1; PaveBlock master;
        std::vector<std::array<double,2>> segs;   // canonical [t_i,t_{i+1}] spans (update_paveblock)
    };
    std::vector<Sec> secs;
    const int NSEC = 48;
    for (const auto& fa : A.m_faces) {
        if (fa.surface_index < 0 || fa.surface_index >= (int)A.m_surfaces.size()) continue;
        const NurbsSurface& sa = A.m_surfaces[fa.surface_index];
        auto abb = aabb_from_surface(sa);
        double am = std::max({abb.second[0]-abb.first[0], abb.second[1]-abb.first[1], abb.second[2]-abb.first[2]}) * 1e-3;
        for (const auto& fb : B.m_faces) {
            if (fb.surface_index < 0 || fb.surface_index >= (int)B.m_surfaces.size()) continue;
            const NurbsSurface& sb = B.m_surfaces[fb.surface_index];
            auto bbb = aabb_from_surface(sb);
            if (!aabb_overlap(abb, bbb, am)) continue;
            for (auto& tr : Intersection::surface_surface(sa, sb, tol)) {
                NurbsCurve c3 = std::get<0>(tr);
                if (!c3.is_valid()) continue;
                Sec s; s.c3d = c3;
                auto [t0, t1] = c3.domain();
                s.t0 = t0; s.t1 = t1;
                std::array<double,6> bb = {1e300,1e300,1e300,-1e300,-1e300,-1e300};
                for (int k = 0; k <= NSEC; ++k) {
                    Point p = c3.point_at(t0 + (t1-t0)*k/NSEC);
                    s.samp.push_back(p);
                    for (int d = 0; d < 3; ++d) { bb[d] = std::min(bb[d], p[d]); bb[d+3] = std::max(bb[d+3], p[d]); }
                }
                s.bb = bb;
                s.closed = c3.point_at(t0).distance(c3.point_at(t1)) < tol;
                secs.push_back(std::move(s));
            }
        }
    }
    if (secs.empty()) { sew_coincident_edges(tol); return; }   // nothing recognised -> legacy merge

    // resolution(C,t): parametric tolerance = tol3d / |C'(t)| (GeomAdaptor::Resolution analogue).
    auto resolution = [&](const NurbsCurve& C, double t) -> double {
        auto d = C.evaluate(t, 1);
        double s = (d.size() > 1) ? d[1].magnitude() : 0.0;
        return (s > 1e-12) ? (tol / s) : tol;
    };
    // ContainsParameter (BOPDS_PaveBlock): is t already represented (within param-tol) among the
    // block's bound + ext paves?
    auto contains_param = [&](const PaveBlock& pb, double t, double ptol) -> bool {
        if (std::abs(t - pb.p1.t) <= ptol || std::abs(t - pb.p2.t) <= ptol) return true;
        for (const auto& e : pb.ext) if (std::abs(t - e.t) <= ptol) return true;
        return false;
    };

    // ---- Phase 2 (paves) ----
    // PutBoundPaveOnCurve: the two curve ends are the natural bound paves of the section curve.
    for (auto& s : secs) { s.master.p1 = Pave{-1, s.t0}; s.master.p2 = Pave{-1, s.t1}; }
    // PutPavesOnCurve: project every existing topology vertex that lies ON a section curve and
    // append it as an interior pave (parametric dedup via resolution). These are the genuine split
    // vertices where the section's arcs meet on this BRep (e.g. the periodic-seam crossings).
    for (int vid = 0; vid < (int)m_topology_vertices.size(); ++vid) {
        int pidx = m_topology_vertices[vid].point_index;
        if (pidx < 0 || pidx >= (int)m_vertices.size()) continue;
        const Point& P = m_vertices[pidx];
        for (auto& s : secs) {
            if (P[0] < s.bb[0]-tol || P[0] > s.bb[3]+tol || P[1] < s.bb[1]-tol ||
                P[1] > s.bb[4]+tol || P[2] < s.bb[2]-tol || P[2] > s.bb[5]+tol) continue;
            double t = s.c3d.closest_parameter(P);
            if (s.c3d.point_at(t).distance(P) > tol) continue;
            double frac = (s.t1 > s.t0) ? (t - s.t0) / (s.t1 - s.t0) : 0.0;
            if (frac <= 1e-6 || frac >= 1.0 - 1e-6) continue;     // bounds already paves
            if (contains_param(s.master, t, resolution(s.c3d, t))) continue;
            s.master.ext.push_back(Pave{vid, t});
        }
    }
    // PutClosingPaveOnCurve: a CLOSED section curve has point_at(t0) ~ point_at(t1); its two bound
    // paves are the same vertex, so the closing pave is the bound pair itself. With K interior
    // paves it yields K segments plus the seam-wrap segment (realised by co-refine's wrap-join);
    // with none it stays one closed edge. Nothing further to append here.

    // ---- Phase 3 (PostTreatFF): intersect section curves among themselves; split at crossings ----
    // GATED on a genuine interior crossing within tol of two DISTINCT section curves (a 3-surface
    // corner). Single-section configs (box-sphere, box-cyl caps) have no crossings -> dormant.
    for (size_t i = 0; i < secs.size(); ++i) {
        for (size_t j = i + 1; j < secs.size(); ++j) {
            const auto& bi = secs[i].bb; const auto& bj = secs[j].bb;
            if (bi[0] > bj[3]+tol || bj[0] > bi[3]+tol || bi[1] > bj[4]+tol ||
                bj[1] > bi[4]+tol || bi[2] > bj[5]+tol || bj[2] > bi[5]+tol) continue;
            auto [ui, uj, dist] = Closest::curve_curve(secs[i].c3d, secs[j].c3d);
            if (dist > tol) continue;
            auto interior = [&](const Sec& s, double tc) {
                double frac = (s.t1 > s.t0) ? (tc - s.t0) / (s.t1 - s.t0) : 0.0;
                return frac > 1e-6 && frac < 1.0 - 1e-6 &&
                       !contains_param(s.master, tc, resolution(s.c3d, tc));
            };
            bool ii = interior(secs[i], ui), ij = interior(secs[j], uj);
            if (!ii && !ij) continue;                              // shared endpoint, not a crossing
            // a brand-new shared corner vertex (3-surface corner); fuse vids by reusing one point.
            Point Pc = secs[i].c3d.point_at(ui);
            int pidx = add_vertex(Pc);
            int vid = (int)m_topology_vertices.size();
            BRepVertex tv; tv.point_index = pidx; m_topology_vertices.push_back(tv);
            if (ii) secs[i].master.ext.push_back(Pave{vid, ui});
            if (ij) secs[j].master.ext.push_back(Pave{vid, uj});
        }
    }

    // ---- update_paveblock (BOPDS_PaveBlock::Update): sort ext+bound paves by parameter and emit
    // consecutive [t_i,t_{i+1}] segments, skipping zero-length (|t1-t2| < pconf) ones. ----
    for (auto& s : secs) {
        std::vector<Pave> all = s.master.ext;
        all.push_back(s.master.p1);
        all.push_back(s.master.p2);
        std::sort(all.begin(), all.end(), [](const Pave& a, const Pave& b){ return a.t < b.t; });
        double pconf = std::max(resolution(s.c3d, 0.5*(s.t0+s.t1)), 1e-9);
        std::vector<double> tp;
        for (const auto& p : all) if (tp.empty() || std::abs(p.t - tp.back()) > pconf) tp.push_back(p.t);
        // assert monotone sorted (mitigation)
        for (size_t k = 1; k < tp.size(); ++k) if (tp[k] < tp[k-1]) tp[k] = tp[k-1];
        for (size_t k = 0; k + 1 < tp.size(); ++k)
            if (tp[k+1] - tp[k] >= pconf) s.segs.push_back({tp[k], tp[k+1]});
    }

    // ---- Phase 4a: a SINGLE co-refine pass so both operands agree on the section's split set
    // (the box full-circle is topologically split at the periodic-side arcs' vertices). ----
    co_refine_coincident_edges(tol);

    // Distance from p to a polyline (point-to-segment).
    auto pt_to_poly = [](const Point& p, const std::vector<Point>& pts) -> double {
        double best = 1e300;
        for (size_t j = 0; j + 1 < pts.size(); ++j) {
            const Point& a = pts[j]; const Point& b = pts[j+1];
            double ex=b[0]-a[0], ey=b[1]-a[1], ez=b[2]-a[2], L2=ex*ex+ey*ey+ez*ez;
            double t = (L2>1e-30) ? ((p[0]-a[0])*ex+(p[1]-a[1])*ey+(p[2]-a[2])*ez)/L2 : 0.0;
            t = std::min(std::max(t,0.0),1.0);
            double cx=a[0]+t*ex, cy=a[1]+t*ey, cz=a[2]+t*ez;
            best = std::min(best, std::sqrt((p[0]-cx)*(p[0]-cx)+(p[1]-cy)*(p[1]-cy)+(p[2]-cz)*(p[2]-cz)));
        }
        return best;
    };
    // Sample the exact section curve C over the sub-arc whose end PARAMETERS are ts,te and that
    // passes through Pm, oriented ts->te. point_at only (no split, unreliable on a closed rational
    // circle) -> a closed full circle yields a clean open arc. ts,te are already C-domain params
    // (snapped to the shared pave set), so the two operands land on IDENTICAL endpoints.
    auto sample_subarc = [&](const NurbsCurve& C, double ts, double te, const Point& Pm) -> NurbsCurve {
        auto [d0, d1] = C.domain(); double rng = d1 - d0;
        if (rng <= 1e-12) return NurbsCurve();
        bool closed = C.point_at(d0).distance(C.point_at(d1)) < tol;
        double tm = C.closest_parameter(Pm);
        const int N = 40;
        std::vector<Point> pts;
        if (closed) {
            auto nrm = [&](double t){ double x = std::fmod(t - d0, rng); if (x < 0) x += rng; return d0 + x; };
            ts = nrm(ts); te = nrm(te); tm = nrm(tm);
            double te_f = (te >= ts) ? te : te + rng;            // forward (increasing) target
            double tm_f = (tm >= ts) ? tm : tm + rng;
            double tend;
            if (tm_f <= te_f + 1e-12) tend = te_f;               // mid lies on the forward arc
            else tend = (te <= ts) ? te : te - rng;              // else go backward
            for (int k = 0; k <= N; ++k) pts.push_back(C.point_at(nrm(ts + (tend - ts) * k / N)));
        } else {
            for (int k = 0; k <= N; ++k) pts.push_back(C.point_at(ts + (te - ts) * k / N));
        }
        if (pts.size() < 2) return NurbsCurve();
        return NurbsCurve::create(false, 1, pts);
    };

    // ---- Phase 4b: re-fit each under-mated section edge to the exact shared sub-arc. The edge's
    // projected endpoint parameters are SNAPPED to the nearest canonical pave parameter (the
    // split-exactly-once set), so the two operands' arcs of a segment land on the IDENTICAL
    // [t_i,t_j] -> bit-identical geometry that sews into one shared edge. ----
    int ne = (int)m_topology_edges.size();
    for (int e = 0; e < ne; ++e) {
        BRepEdge& E = m_topology_edges[e];
        if ((int)E.trim_indices.size() >= 2) continue;            // already mated -> not a section edge
        int ci = E.curve_3d_index;
        if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
        const NurbsCurve C = m_curves_3d[ci];
        if (!C.is_valid()) continue;
        auto [et0, et1] = C.domain();
        const int M = 16;
        std::vector<Point> es; std::array<double,6> ebb = {1e300,1e300,1e300,-1e300,-1e300,-1e300};
        for (int k = 0; k <= M; ++k) {
            Point p = C.point_at(et0 + (et1-et0)*k/M); es.push_back(p);
            for (int d = 0; d < 3; ++d) { ebb[d]=std::min(ebb[d],p[d]); ebb[d+3]=std::max(ebb[d+3],p[d]); }
        }
        // Match to the single exact section curve that contains the WHOLE edge (every sample on it).
        int best = -1;
        for (size_t si = 0; si < secs.size(); ++si) {
            const auto& B6 = secs[si].bb;
            if (ebb[0] > B6[3]+tol || B6[0] > ebb[3]+tol || ebb[1] > B6[4]+tol ||
                B6[1] > ebb[4]+tol || ebb[2] > B6[5]+tol || B6[2] > ebb[5]+tol) continue;
            bool on = true;
            for (const auto& p : es) if (pt_to_poly(p, secs[si].samp) > tol) { on = false; break; }
            if (on) { best = (int)si; break; }
        }
        if (best < 0) continue;
        Sec& S = secs[best];
        double ts = S.c3d.closest_parameter(es.front());
        double te = S.c3d.closest_parameter(es.back());
        // snap each endpoint param to the nearest canonical pave parameter, but ONLY if it lands
        // within the parametric snap window (else keep the edge's own projected param) -- a
        // full-circle edge whose ends sit at the seam stays a full circle (snap is a no-op).
        auto snap = [&](double t) -> double {
            double best_t = t, bd = 1e300;
            auto consider = [&](double pt){ double d = std::abs(pt - t); if (d < bd) { bd = d; best_t = pt; } };
            consider(S.t0); consider(S.t1);
            for (const auto& sg : S.segs) { consider(sg[0]); consider(sg[1]); }
            double win = std::max(resolution(S.c3d, t) * 4.0, (S.t1 - S.t0) * 1e-4);
            return (bd <= win) ? best_t : t;
        };
        ts = snap(ts); te = snap(te);
        NurbsCurve sub = sample_subarc(S.c3d, ts, te, es[M/2]);
        // guard: only substitute if the result still hugs the original edge (never worsen sewing).
        if (sub.is_valid()) {
            auto [s0, s1] = sub.domain();
            bool ok = true;
            for (int k = 0; k <= 8; ++k) {
                Point sp = sub.point_at(s0 + (s1-s0)*k/8);
                if (pt_to_poly(sp, es) > std::max(tol, 1e-9)) { ok = false; break; }
            }
            if (ok) m_curves_3d[ci] = sub;
        }
    }

    // ---- Phase 4c: merge the now-identical coincident arcs into one shared edge (two trims). ----
    sew_coincident_edges(tol);
}

void BRep::sew_coincident_edges(double tol) {
    double diag;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& p : m_vertices) {
            xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
            xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        }
        diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        if (diag <= 0) diag = 1.0;
    }
    // Two imprinted intersection edges (a circle imprinted independently on A and B) are the
    // same edge but discretised as polylines with different phase/sampling (one may be an exact
    // rational circle, the other a degree-3 spline fit), so their vertices do NOT coincide
    // point-for-point. We measure a point-to-POLYLINE (segment) directed Hausdorff both ways: a
    // vertex of one polyline lands within the other's chord sagitta. The default tolerance is
    // feature-relative (diag*5e-3) so it comfortably absorbs that sagitta yet stays far below
    // the spacing of the kernel solids' distinct edges (O(diag) apart), avoiding false merges.
    if (tol <= 0.0) tol = diag * 5e-3;
    int ne = (int)m_topology_edges.size();
    const int NS = 16;
    std::vector<std::vector<Point>> samp(ne);
    std::vector<std::array<double,6>> bbox(ne);  // xmn,ymn,zmn,xmx,ymx,zmx per edge
    // Only UNDER-mated edges (fewer than 2 trims) are sewing candidates: a shared intersection
    // curve is imprinted as a 1-trim edge on each side and the two halves must merge into one
    // 2-trim edge. Edges that are already 2-trim (the solids' own box/cyl/sphere edges) are
    // watertight and cannot legitimately coincide with another edge, so we skip sampling AND
    // comparing them -- turning the O(ne^2) sew into O(k^2) over the few intersection edges.
    auto is_candidate = [&](int ei) { return (int)m_topology_edges[ei].trim_indices.size() < 2; };
    for (int ei = 0; ei < ne; ++ei) {
        if (!is_candidate(ei)) continue;
        int ci = m_topology_edges[ei].curve_3d_index;
        if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
        auto [t0, t1] = m_curves_3d[ci].domain();
        std::array<double,6> bb = {1e300,1e300,1e300,-1e300,-1e300,-1e300};
        for (int k = 0; k <= NS; ++k) {
            Point p = m_curves_3d[ci].point_at(t0 + (t1 - t0) * k / NS);
            samp[ei].push_back(p);
            for (int d = 0; d < 3; ++d) { bb[d] = std::min(bb[d], p[d]); bb[d+3] = std::max(bb[d+3], p[d]); }
        }
        bbox[ei] = bb;
    }
    // Cheap reject: edge AABBs (expanded by tol) must overlap to possibly coincide.
    auto bbox_far = [&](int i, int j) -> bool {
        const auto& a = bbox[i]; const auto& b = bbox[j];
        return a[0] > b[3]+tol || b[0] > a[3]+tol || a[1] > b[4]+tol || b[1] > a[4]+tol
            || a[2] > b[5]+tol || b[2] > a[5]+tol;
    };
    // Distance from point p to the polyline through pts (point-to-segment, not point-to-vertex).
    auto pt_to_polyline = [](const Point& p, const std::vector<Point>& pts) -> double {
        double best = 1e300;
        for (size_t j = 0; j + 1 < pts.size(); ++j) {
            const Point& a = pts[j]; const Point& b = pts[j + 1];
            double ex = b[0]-a[0], ey = b[1]-a[1], ez = b[2]-a[2];
            double L2 = ex*ex + ey*ey + ez*ez;
            double t = (L2 > 1e-30) ? ((p[0]-a[0])*ex+(p[1]-a[1])*ey+(p[2]-a[2])*ez)/L2 : 0.0;
            t = std::min(std::max(t, 0.0), 1.0);
            double cx = a[0]+t*ex, cy = a[1]+t*ey, cz = a[2]+t*ez;
            double d = std::sqrt((p[0]-cx)*(p[0]-cx)+(p[1]-cy)*(p[1]-cy)+(p[2]-cz)*(p[2]-cz));
            best = std::min(best, d);
        }
        return best;
    };
    // Directed point-to-polyline Hausdorff both ways, but EARLY-EXIT as soon as any sample
    // exceeds `tol` (the curves are then provably not coincident). Identical accept/reject to a
    // full max-Hausdorff < tol, but rejection is O(1) instead of O(NS^2) for the common case.
    // The midpoint is checked first because non-coincident edges that share an endpoint diverge
    // most in the middle, so it rejects fastest.
    auto coincident_within = [&](const std::vector<Point>& a, const std::vector<Point>& b) -> bool {
        if (a.size() < 2 || b.size() < 2) return false;
        if (pt_to_polyline(a[a.size()/2], b) > tol) return false;
        if (pt_to_polyline(b[b.size()/2], a) > tol) return false;
        for (const auto& p : a) if (pt_to_polyline(p, b) > tol) return false;
        for (const auto& p : b) if (pt_to_polyline(p, a) > tol) return false;
        return true;
    };
    std::vector<int> rep(ne, -1);
    std::vector<int> reps;
    for (int ei = 0; ei < ne; ++ei) {
        if (samp[ei].empty()) { rep[ei] = ei; reps.push_back(ei); continue; }
        for (int r : reps)
            if (!samp[r].empty() && !bbox_far(ei, r) && coincident_within(samp[ei], samp[r])) { rep[ei] = r; break; }
        if (rep[ei] < 0) { rep[ei] = ei; reps.push_back(ei); }
    }
    std::map<int, int> old2new;
    std::vector<BRepEdge> newedges;
    for (int r : reps) {
        old2new[r] = (int)newedges.size();
        BRepEdge e = m_topology_edges[r];
        e.trim_indices.clear();
        newedges.push_back(e);
    }
    for (int ti = 0; ti < (int)m_trims.size(); ++ti) {
        int oe = m_trims[ti].edge_index;
        if (oe < 0 || oe >= ne) continue;
        int ni = old2new[rep[oe]];
        m_trims[ti].edge_index = ni;
        newedges[ni].trim_indices.push_back(ti);
    }
    m_topology_edges = std::move(newedges);
    for (auto& v : m_topology_vertices) v.edge_indices.clear();
    for (int ei = 0; ei < (int)m_topology_edges.size(); ++ei) {
        int sv = m_topology_edges[ei].start_vertex;
        int ev = m_topology_edges[ei].end_vertex;
        if (sv >= 0 && sv < (int)m_topology_vertices.size()) m_topology_vertices[sv].edge_indices.push_back(ei);
        if (ev != sv && ev >= 0 && ev < (int)m_topology_vertices.size()) m_topology_vertices[ev].edge_indices.push_back(ei);
    }
}

namespace {
// A recognized analytic solid for O(1) point classification (skips the tessellate+ray-cast).
// Kind 1 = convex polyhedron (box/beam/plate): intersection of outward half-spaces.
// Kind 2 = cylinder: within radius of an axis segment. Kind 0 = unrecognized (caller falls
// back to the mesh ray-cast). Recognition self-VERIFIES; on any mismatch it returns kind 0,
// so a wrong guess can never produce a wrong classification -- only a slower (mesh) one.
struct PrimSolid {
    int kind = 0;
    double tol = 0;
    std::vector<std::array<double,4>> hs;  // half-spaces: inside iff n.p <= d
    Point ca{0,0,0}; Vector cd{0,0,1}; double ch = 0, cr = 0;  // cylinder axis/length/radius
};

static bool srf_is_planar(const NurbsSurface& s) {
    auto du = s.domain(0); auto dv = s.domain(1);
    Vector n0 = s.normal_at(0.5*(du.first+du.second), 0.5*(dv.first+dv.second));
    const double uu[2] = {0.25, 0.75}, vv[2] = {0.3, 0.8};
    for (int i = 0; i < 2; ++i) {
        Vector n = s.normal_at(du.first+(du.second-du.first)*uu[i], dv.first+(dv.second-dv.first)*vv[i]);
        Vector c(n0[1]*n[2]-n0[2]*n[1], n0[2]*n[0]-n0[0]*n[2], n0[0]*n[1]-n0[1]*n[0]);
        if (c.magnitude() > 1e-7) return false;
    }
    return true;
}

static PrimSolid recognize_solid(const BRep& X) {
    PrimSolid ps;
    if (X.m_faces.empty() || X.m_surfaces.empty()) return ps;
    // Derive geometry from the SURFACES (authoritative) rather than m_vertices, which can be
    // stale relative to the surfaces after transformed(). Sample each face's surface at its
    // corners + centre; these points define the bbox, an interior reference point, and the
    // convexity check.
    std::vector<Point> spts;
    for (const auto& f : X.m_faces) {
        if (f.surface_index < 0 || f.surface_index >= (int)X.m_surfaces.size()) return ps;
        const NurbsSurface& s = X.m_surfaces[f.surface_index];
        auto du = s.domain(0); auto dv = s.domain(1);
        for (double a : {0.0, 0.5, 1.0}) for (double b : {0.0, 0.5, 1.0})
            spts.push_back(s.point_at(du.first+(du.second-du.first)*a, dv.first+(dv.second-dv.first)*b));
    }
    if (spts.empty()) return ps;
    double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
    double cx=0,cy=0,cz=0;
    for (const auto& p : spts) {
        xmn=std::min(xmn,p[0]); ymn=std::min(ymn,p[1]); zmn=std::min(zmn,p[2]);
        xmx=std::max(xmx,p[0]); ymx=std::max(ymx,p[1]); zmx=std::max(zmx,p[2]);
        cx+=p[0]; cy+=p[1]; cz+=p[2];
    }
    int nv=(int)spts.size(); Point C(cx/nv, cy/nv, cz/nv);
    double diag = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
    if (diag < 1e-12) return ps;
    double tol = diag * 1e-6;
    ps.tol = tol;

    // All-planar -> convex polyhedron (box/beam/plate). Build outward half-spaces, verify convex.
    bool all_planar = true;
    for (const auto& f : X.m_faces) {
        if (f.surface_index < 0 || f.surface_index >= (int)X.m_surfaces.size()) { all_planar=false; break; }
        if (!srf_is_planar(X.m_surfaces[f.surface_index])) { all_planar=false; break; }
    }
    if (all_planar) {
        for (const auto& f : X.m_faces) {
            const NurbsSurface& s = X.m_surfaces[f.surface_index];
            auto du = s.domain(0); auto dv = s.domain(1);
            Point q = s.point_at(0.5*(du.first+du.second), 0.5*(dv.first+dv.second));
            Vector n = s.normal_at(0.5*(du.first+du.second), 0.5*(dv.first+dv.second));
            if (n[0]*(C[0]-q[0]) + n[1]*(C[1]-q[1]) + n[2]*(C[2]-q[2]) > 0) n = Vector(-n[0],-n[1],-n[2]);
            ps.hs.push_back({n[0], n[1], n[2], n[0]*q[0]+n[1]*q[1]+n[2]*q[2]});
        }
        for (const auto& vtx : spts)
            for (const auto& h : ps.hs)
                if (h[0]*vtx[0]+h[1]*vtx[1]+h[2]*vtx[2] > h[3] + tol*50) return PrimSolid{};  // not convex
        ps.kind = 1;
        return ps;
    }

    // Sphere: a single (non-planar) face. Fit a centre by least squares from a UV grid of
    // surface points (2(Pi-P0).C = |Pi|^2-|P0|^2), then verify all points are equidistant.
    if (X.m_faces.size() == 1) {
        const NurbsSurface& s = X.m_surfaces[X.m_faces[0].surface_index];
        auto du = s.domain(0); auto dv = s.domain(1);
        std::vector<Point> sp;
        for (int i = 1; i <= 5; ++i) for (int j = 1; j <= 5; ++j)
            sp.push_back(s.point_at(du.first+(du.second-du.first)*i/6.0, dv.first+(dv.second-dv.first)*j/6.0));
        double A[3][3] = {{0,0,0},{0,0,0},{0,0,0}}, b[3] = {0,0,0};
        const Point& p0 = sp[0]; double p0d = p0[0]*p0[0]+p0[1]*p0[1]+p0[2]*p0[2];
        for (size_t i = 1; i < sp.size(); ++i) {
            double r0=2*(sp[i][0]-p0[0]), r1=2*(sp[i][1]-p0[1]), r2=2*(sp[i][2]-p0[2]);
            double rhs = sp[i][0]*sp[i][0]+sp[i][1]*sp[i][1]+sp[i][2]*sp[i][2] - p0d;
            double row[3] = {r0,r1,r2};
            for (int a=0;a<3;++a){ for (int c=0;c<3;++c) A[a][c]+=row[a]*row[c]; b[a]+=row[a]*rhs; }
        }
        // Solve 3x3 A.c=b by Cramer's rule.
        auto det3=[](double m[3][3]){return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
            - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0]) + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);};
        double D = det3(A);
        if (std::abs(D) > 1e-12) {
            double c[3];
            for (int k=0;k<3;++k){ double M[3][3]; for(int a=0;a<3;++a)for(int cc=0;cc<3;++cc)M[a][cc]=(cc==k)?b[a]:A[a][cc]; c[k]=det3(M)/D; }
            Point center(c[0],c[1],c[2]);
            double rs=0; for (auto&p:sp) rs+=center.distance(p); double radius=rs/sp.size();
            bool ok = radius > tol;
            for (auto&p:sp) if (std::abs(center.distance(p)-radius) > diag*1e-3) { ok=false; break; }
            if (ok) { ps.kind=3; ps.ca=center; ps.cr=radius; return ps; }
        }
    }

    // Cylinder: exactly 2 planar caps + 1 curved lateral. The two circular cap edges give the
    // axis (cap-centre to cap-centre) and radius; verify the lateral face lies on that cylinder.
    std::vector<int> planar, curved;
    for (int fi = 0; fi < (int)X.m_faces.size(); ++fi) {
        int si = X.m_faces[fi].surface_index;
        if (si < 0) return ps;
        (srf_is_planar(X.m_surfaces[si]) ? planar : curved).push_back(fi);
    }
    if (planar.size() == 2 && curved.size() == 1) {
        // Cap centre/radius from each planar face's outer-loop boundary (sampled in 3D).
        auto cap_circle = [&](int fi, Point& center, double& radius) -> bool {
            const auto& face = X.m_faces[fi];
            const NurbsSurface& s = X.m_surfaces[face.surface_index];
            std::vector<Point> bpts;
            for (int li : face.loop_indices) {
                if (li < 0 || li >= (int)X.m_loops.size()) continue;
                if (X.m_loops[li].type != BRepLoopType::Outer) continue;
                for (int ti : X.m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                    int c2 = X.m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)X.m_curves_2d.size()) continue;
                    const NurbsCurve& pc = X.m_curves_2d[c2];
                    auto dc = pc.domain();
                    for (int k = 0; k < 16; ++k) {
                        Point uv = pc.point_at(dc.first + (dc.second-dc.first)*k/16.0);
                        bpts.push_back(s.point_at(uv[0], uv[1]));
                    }
                }
            }
            if (bpts.size() < 6) return false;
            double mx=0,my=0,mz=0; for (auto&p:bpts){mx+=p[0];my+=p[1];mz+=p[2];}
            center = Point(mx/bpts.size(), my/bpts.size(), mz/bpts.size());
            double rs=0; for (auto&p:bpts) rs += center.distance(p);
            radius = rs/bpts.size();
            for (auto&p:bpts) if (std::abs(center.distance(p)-radius) > diag*1e-3) return false;  // not a circle
            return true;
        };
        Point c0, c1; double r0, r1;
        if (cap_circle(planar[0], c0, r0) && cap_circle(planar[1], c1, r1) &&
            std::abs(r0 - r1) < diag*1e-3) {
            double h = c0.distance(c1);
            if (h > tol) {
                Vector d((c1[0]-c0[0])/h, (c1[1]-c0[1])/h, (c1[2]-c0[2])/h);
                // Verify the lateral face sits on this cylinder.
                const NurbsSurface& L = X.m_surfaces[X.m_faces[curved[0]].surface_index];
                auto du = L.domain(0); auto dv = L.domain(1); bool ok = true;
                for (int i = 0; i <= 3 && ok; ++i) for (int j = 0; j <= 3 && ok; ++j) {
                    Point p = L.point_at(du.first+(du.second-du.first)*i/3.0, dv.first+(dv.second-dv.first)*j/3.0);
                    Vector w(p[0]-c0[0], p[1]-c0[1], p[2]-c0[2]);
                    double t = w[0]*d[0]+w[1]*d[1]+w[2]*d[2];
                    double rad = std::sqrt(std::max(0.0, w.magnitude()*w.magnitude() - t*t));
                    if (std::abs(rad - r0) > diag*1e-3 || t < -diag*1e-3 || t > h+diag*1e-3) ok = false;
                }
                if (ok) { ps.kind=2; ps.ca=c0; ps.cd=d; ps.ch=h; ps.cr=r0; return ps; }
            }
        }
    }
    return ps;
}

static bool inside_prim(const PrimSolid& ps, const Point& p, double tol) {
    if (ps.kind == 1) {
        for (const auto& h : ps.hs)
            if (h[0]*p[0]+h[1]*p[1]+h[2]*p[2] > h[3] + tol) return false;
        return true;
    }
    if (ps.kind == 2) {
        Vector w(p[0]-ps.ca[0], p[1]-ps.ca[1], p[2]-ps.ca[2]);
        double t = w[0]*ps.cd[0]+w[1]*ps.cd[1]+w[2]*ps.cd[2];
        if (t < -tol || t > ps.ch + tol) return false;
        double rad = std::sqrt(std::max(0.0, w.magnitude()*w.magnitude() - t*t));
        return rad <= ps.cr + tol;
    }
    if (ps.kind == 3) {  // sphere: centre ps.ca, radius ps.cr
        return ps.ca.distance(p) <= ps.cr + tol;
    }
    return false;
}
}  // namespace

BRep BRep::boolean(const BRep& other, BooleanOp op, double tolerance) const {
    // Optional phase profiling (set SESSION_BOOL_PROFILE=1). Prints per-phase microseconds.
    static const bool s_prof = (std::getenv("SESSION_BOOL_PROFILE") != nullptr);
    auto t_now = []{ return std::chrono::high_resolution_clock::now(); };
    auto t_us = [](auto a, auto b){ return std::chrono::duration<double, std::micro>(b - a).count(); };
    auto t_start = t_now();
    auto tp = t_now();
    if (s_prof) std::fprintf(stderr, "[bool-prof] === A=%dfaces B=%dfaces op=%d ===\n",
                             (int)m_faces.size(), (int)other.m_faces.size(), (int)op);
    auto lap = [&](const char* what){ if (s_prof) { auto n = t_now(); std::fprintf(stderr, "[bool-prof]   %-16s %8.1f us\n", what, t_us(tp, n)); tp = n; } };

    // Imprint each solid against the other (split faces along the SSI curves).
    BRep A2 = split_by_brep(other, tolerance);                 lap("splitA");
    BRep B2 = other.split_by_brep(*this, tolerance);           lap("splitB");
    // Classify fragments against the OTHER solid. The operands are typically recognized
    // primitives (box/beam/cylinder) -> test point-in-solid analytically in O(1) and skip the
    // tessellate+ray-cast entirely. Only build a mesh for an operand we could not recognize.
    PrimSolid primA = recognize_solid(*this);
    PrimSolid primB = recognize_solid(other);
    Mesh meshA, meshB;
    if (primA.kind == 0) meshA = mesh();
    if (primB.kind == 0) meshB = other.mesh();                 lap("recognize+mesh");

    BRep result;
    result.name = "boolean";

    // A 3D point strictly inside face fi of X (CDT the UV trim region, centroid of a triangle).
    auto face_sample = [&](const BRep& X, int fi) -> Point {
        const auto& face = X.m_faces[fi];
        const NurbsSurface& srf = X.m_surfaces[face.surface_index];
        std::vector<Polyline> outers, inners;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)X.m_loops.size()) continue;
            std::vector<Point> pts;
            for (int ti : X.m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)X.m_trims.size()) continue;
                int c2 = X.m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)X.m_curves_2d.size()) continue;
                const NurbsCurve& pc = X.m_curves_2d[c2];
                auto dc = pc.domain();
                int n = std::max(pc.cv_count() * 3, 6);
                for (int i = 0; i < n; ++i) {
                    Point uv = pc.point_at(dc.first + (dc.second - dc.first) * i / n);
                    pts.push_back(Point(uv[0], uv[1], 0));
                }
            }
            if (pts.size() < 3) continue;
            if (X.m_loops[li].type == BRepLoopType::Outer) outers.emplace_back(pts);
            else inners.emplace_back(pts);
        }
        auto fallback = [&]() {
            auto [u0,u1] = srf.domain(0); auto [v0,v1] = srf.domain(1);
            return srf.point_at(0.5*(u0+u1), 0.5*(v0+v1));
        };
        if (outers.empty()) return fallback();
        std::vector<Polyline> all; all.push_back(outers[0]);
        for (auto& in : inners) all.push_back(in);
        std::vector<std::array<int,3>> tris;
        try { tris = RemeshCDT::triangulate(all); } catch (...) { return fallback(); }
        std::vector<Point> flat;
        for (auto& pl : all) { auto pp = pl.get_points(); for (auto& p : pp) flat.push_back(p); }
        if (tris.empty() || flat.empty()) return fallback();
        // Sample the centroid of the LARGEST triangle: it sits well inside the trim region (the
        // first triangle is often a thin sliver against the boundary, whose centroid grazes the
        // other solid's surface and misclassifies).
        int best = -1; double best_area = -1.0;
        for (size_t ti = 0; ti < tris.size(); ++ti) {
            const auto& t = tris[ti];
            if (t[0] < 0 || t[2] >= (int)flat.size()) continue;
            double ar = std::abs((flat[t[1]][0]-flat[t[0]][0])*(flat[t[2]][1]-flat[t[0]][1])
                               - (flat[t[2]][0]-flat[t[0]][0])*(flat[t[1]][1]-flat[t[0]][1])) * 0.5;
            if (ar > best_area) { best_area = ar; best = (int)ti; }
        }
        if (best < 0) return fallback();
        const auto& t = tris[best];
        double cu = (flat[t[0]][0] + flat[t[1]][0] + flat[t[2]][0]) / 3.0;
        double cv = (flat[t[0]][1] + flat[t[1]][1] + flat[t[2]][1]) / 3.0;
        return srf.point_at(cu, cv);
    };

    // Classify each imprinted fragment inside/outside the other solid, select per op.
    std::vector<int> keptA, keptB; std::vector<bool> revB;
    auto classify = [&](const BRep& X2, const BRep& solid, const Mesh& solid_mesh,
                        const PrimSolid& prim, bool is_first,
                        std::vector<int>& kept, std::vector<bool>* rev) {
        for (int fi = 0; fi < (int)X2.m_faces.size(); ++fi) {
            if (X2.m_faces[fi].surface_index < 0) continue;
            Point sp = face_sample(X2, fi);
            bool inside = prim.kind ? inside_prim(prim, sp, prim.tol)
                                    : solid.contains_point(solid_mesh, sp);
            bool keep = false, r = false;
            if (op == BooleanOp::Union) keep = !inside;
            else if (op == BooleanOp::Intersection) keep = inside;
            else { if (is_first) keep = !inside; else { keep = inside; r = true; } }
            if (keep) { kept.push_back(fi); if (rev) rev->push_back(r); }
        }
    };
    classify(A2, other, meshB, primB, true, keptA, nullptr);
    classify(B2, *this, meshA, primA, false, keptB, &revB);   lap("classify");

    // Select faces WITH their already-mated topology (subset preserves shared edges), then
    // combine the two sides and sew only the A<->B intersection edges. This keeps each
    // solid's internal box/cylinder edges mated -- the key to a watertight result.
    BRep subA = A2.subset(keptA);
    BRep subB = B2.subset(keptB);
    for (size_t k = 0; k < revB.size() && k < subB.m_faces.size(); ++k)
        if (revB[k]) subB.m_faces[k].reversed = !subB.m_faces[k].reversed;

    result = subA;
    result.name = "boolean";
    int voff=(int)result.m_vertices.size(), tvoff=(int)result.m_topology_vertices.size();
    int soff=(int)result.m_surfaces.size(), c2off=(int)result.m_curves_2d.size();
    int c3off=(int)result.m_curves_3d.size(), eoff=(int)result.m_topology_edges.size();
    int loff=(int)result.m_loops.size(), foff=(int)result.m_faces.size(), toff=(int)result.m_trims.size();
    for (auto& p : subB.m_vertices) result.m_vertices.push_back(p);
    for (auto& s : subB.m_surfaces) result.m_surfaces.push_back(s);
    for (auto& c : subB.m_curves_2d) result.m_curves_2d.push_back(c);
    for (auto& c : subB.m_curves_3d) result.m_curves_3d.push_back(c);
    for (auto tv : subB.m_topology_vertices) { tv.point_index += voff; tv.edge_indices.clear(); result.m_topology_vertices.push_back(tv); }
    for (auto e : subB.m_topology_edges) {
        if (e.curve_3d_index>=0) e.curve_3d_index += c3off;
        if (e.start_vertex>=0) e.start_vertex += tvoff;
        if (e.end_vertex>=0) e.end_vertex += tvoff;
        e.trim_indices.clear();
        result.m_topology_edges.push_back(e);
    }
    for (auto t : subB.m_trims) {
        if (t.curve_2d_index>=0) t.curve_2d_index += c2off;
        if (t.edge_index>=0) t.edge_index += eoff;
        if (t.loop_index>=0) t.loop_index += loff;
        result.m_trims.push_back(t);
    }
    for (auto lp : subB.m_loops) {
        for (auto& ti : lp.trim_indices) ti += toff;
        if (lp.face_index>=0) lp.face_index += foff;
        result.m_loops.push_back(lp);
    }
    for (auto f : subB.m_faces) {
        if (f.surface_index>=0) f.surface_index += soff;
        for (auto& li : f.loop_indices) li += loff;
        result.m_faces.push_back(f);
    }
    for (auto& e : result.m_topology_edges) e.trim_indices.clear();
    for (int ti=0; ti<(int)result.m_trims.size(); ++ti) {
        int ei = result.m_trims[ti].edge_index;
        if (ei>=0 && ei<(int)result.m_topology_edges.size()) result.m_topology_edges[ei].trim_indices.push_back(ti);
    }

    // Resolve T-junctions first: a face whose boundary run was split (e.g. by an arrangement
    // artifact) leaves a long edge on the adjacent face spanning several shorter ones. Imprint
    // splits the long edge at those interior vertices so the pieces can mate.
    lap("combine");
    result.imprint_edges();                                   lap("imprint_edges");
    // BUILDSPEC P0: shared section-edge backbone, gated by SESSION_BOOL_SHARED_EDGES. When set,
    // recompute the A&B section curve ONCE and make each operand's section arcs reference the
    // EXACT sub-arc of that single curve, then merge -> one shared edge per section (watertight by
    // construction). When UNSET, the proven legacy path (co-refine + Hausdorff sew) runs UNCHANGED.
    static const bool s_shared = (std::getenv("SESSION_BOOL_SHARED_EDGES") != nullptr);
    if (s_shared) {
        result.make_shared_section_edges(*this, other);       lap("shared_section");
        return result;
    }
    // Co-refine the A<->B section: where one operand imprinted the shared curve as a single closed
    // circle and the other as 2+ arcs (periodic-seam straddle) -- or as partially-overlapping arcs --
    // split the longer at the shorter's endpoints so they mate 1:1. Strictly coincidence-gated, so
    // each solid's own edges are untouched. This is the OCCT "shared section edge" guarantee done as
    // a co-refinement: after it, sew merges segments that are arc-for-arc identical.
    if (!std::getenv("SESSION_NO_COREFINE")) result.co_refine_coincident_edges();
    lap("co_refine");
    // Geometric sewing: the intersection curve is imprinted independently on A and B (a
    // closed self-loop on one, a seam-anchored loop on the other), so their edges share no
    // endpoints and the position-keyed emap cannot mate them. Merge edges whose 3D curves
    // trace the SAME point set (Hausdorff ~ 0), regardless of start point / discretization,
    // into one mated edge -- helps make the result a watertight solid.
    result.sew_coincident_edges();                            lap("sew");
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Meshing
///////////////////////////////////////////////////////////////////////////////////////////

Mesh BRep::mesh() const {
    int nf = (int)m_faces.size();

    // Phase 1: Classify faces as direct (RemeshNurbsSurfaceGrid) or CDT
    std::vector<bool> face_direct(nf, false);
    for (int fi = 0; fi < nf; ++fi) {
        const auto& face = m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        bool has_inner = false, all_linear = true;
        std::vector<Point> outer_pts;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const auto& loop = m_loops[li];
            if (loop.type == BRepLoopType::Inner) has_inner = true;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                const auto& trim = m_trims[ti];
                if (trim.curve_2d_index < 0 || trim.curve_2d_index >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& crv = m_curves_2d[trim.curve_2d_index];
                if (crv.degree() > 1 || crv.is_rational()) all_linear = false;
                if (loop.type == BRepLoopType::Outer) {
                    if (crv.degree() <= 1 && !crv.is_rational())
                        for (int k = 0; k < crv.cv_count() - 1; ++k) outer_pts.push_back(crv.get_cv(k));
                }
            }
        }
        bool direct = !has_inner && all_linear;
        if (direct && !outer_pts.empty()) {
            auto [u0, u1] = srf.domain(0);
            auto [v0, v1] = srf.domain(1);
            double tol = std::max(u1 - u0, v1 - v0) * 0.01;
            double bb_umin = 1e30, bb_umax = -1e30, bb_vmin = 1e30, bb_vmax = -1e30;
            for (const auto& p : outer_pts) {
                bb_umin = std::min(bb_umin, p[0]); bb_umax = std::max(bb_umax, p[0]);
                bb_vmin = std::min(bb_vmin, p[1]); bb_vmax = std::max(bb_vmax, p[1]);
            }
            if (std::abs(bb_umin - u0) > tol || std::abs(bb_umax - u1) > tol ||
                std::abs(bb_vmin - v0) > tol || std::abs(bb_vmax - v1) > tol)
                direct = false;
        }
        face_direct[fi] = direct;
    }

    // Phase 2: Mesh direct faces, extract boundary 3D points for shared edges
    std::vector<Mesh> fmesh(nf);
    std::map<int, std::vector<Point>> edge_bnd;

    for (int fi = 0; fi < nf; ++fi) {
        if (!face_direct[fi]) continue;
        const auto& face = m_faces[fi];
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        fmesh[fi] = srf.mesh();

        auto [u0, u1] = srf.domain(0);
        auto [v0, v1] = srf.domain(1);
        double utol = (u1 - u0) * 0.001, vtol = (v1 - v0) * 0.001;

        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            for (int ti : m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int eidx = m_trims[ti].edge_index;
                if (eidx < 0 || eidx >= (int)m_topology_edges.size()) continue;
                if (edge_bnd.count(eidx)) continue;

                // Only extract if edge is shared with a CDT face
                bool shared = false;
                for (int oti : m_topology_edges[eidx].trim_indices) {
                    if (oti == ti) continue;
                    int oli = m_trims[oti].loop_index;
                    if (oli < 0 || oli >= (int)m_loops.size()) continue;
                    int ofi = m_loops[oli].face_index;
                    if (ofi >= 0 && ofi < nf && !face_direct[ofi]) { shared = true; break; }
                }
                if (!shared) continue;

                // Determine which UV boundary this trim lies on
                int c2di = m_trims[ti].curve_2d_index;
                if (c2di < 0 || c2di >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& c2d = m_curves_2d[c2di];
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
                    if (at_v0 && std::abs(iv->second - v0) < vtol * 0.1)
                        pts.push_back({iu->second, vd.position()});
                    else if (at_v1 && std::abs(iv->second - v1) < vtol * 0.1)
                        pts.push_back({iu->second, vd.position()});
                    else if (at_u0 && std::abs(iu->second - u0) < utol * 0.1)
                        pts.push_back({iv->second, vd.position()});
                    else if (at_u1 && std::abs(iu->second - u1) < utol * 0.1)
                        pts.push_back({iv->second, vd.position()});
                }
                std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
                if (pts.size() >= 2) {
                    std::vector<Point> bnd;
                    for (auto& [p, pt] : pts) bnd.push_back(pt);
                    edge_bnd[eidx] = bnd;
                }
            }
        }
    }

    // Phase 3: Mesh CDT faces, using matched boundary points for shared edges
    for (int fi = 0; fi < nf; ++fi) {
        if (face_direct[fi]) continue;
        const auto& face = m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];

        // Bilinear 3D→UV projection (works for planar cap surfaces)
        Point p00 = srf.get_cv(0, 0), p10 = srf.get_cv(1, 0), p01 = srf.get_cv(0, 1);
        double eux = p10[0]-p00[0], euy = p10[1]-p00[1], euz = p10[2]-p00[2];
        double evx = p01[0]-p00[0], evy = p01[1]-p00[1], evz = p01[2]-p00[2];
        double eu2 = eux*eux+euy*euy+euz*euz, ev2 = evx*evx+evy*evy+evz*evz;
        bool can_project = (srf.degree(0) == 1 && srf.degree(1) == 1 && eu2 > 1e-28 && ev2 > 1e-28);

        NurbsSurfaceTrimmed ts;
        ts.m_surface = srf;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const auto& loop = m_loops[li];
            std::vector<Point> loop_pts;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                const auto& trim = m_trims[ti];
                if (trim.type == BRepTrimType::Singular) continue;
                int eidx = trim.edge_index;

                if (can_project && eidx >= 0 && edge_bnd.count(eidx)) {
                    // Project matched 3D boundary points to this face's UV
                    const auto& bnd = edge_bnd[eidx];
                    for (const auto& pt : bnd) {
                        double dx = pt[0]-p00[0], dy = pt[1]-p00[1], dz = pt[2]-p00[2];
                        double u = (dx*eux+dy*euy+dz*euz) / eu2;
                        double v = (dx*evx+dy*evy+dz*evz) / ev2;
                        loop_pts.push_back(Point(u, v, 0));
                    }
                } else {
                    if (trim.curve_2d_index < 0 || trim.curve_2d_index >= (int)m_curves_2d.size()) continue;
                    const NurbsCurve& crv = m_curves_2d[trim.curve_2d_index];
                    if (crv.degree() <= 1 && !crv.is_rational()) {
                        for (int k = 0; k < crv.cv_count() - 1; ++k)
                            loop_pts.push_back(crv.get_cv(k));
                    } else {
                        int n = std::max(crv.cv_count() * 4, 16);
                        auto [pts, params] = crv.divide_by_count(n);
                        for (int k = 0; k < (int)pts.size() - 1; ++k)
                            loop_pts.push_back(pts[k]);
                    }
                }
            }
            if (loop_pts.size() >= 3) {
                NurbsCurve loop_crv = NurbsCurve::create(true, 1, loop_pts);
                if (loop.type == BRepLoopType::Outer)
                    ts.m_outer_loop = loop_crv;
                else
                    ts.m_inner_loops.push_back(loop_crv);
            }
        }
        fmesh[fi] = ts.mesh();
    }

    // Phase 4: Combine all face meshes
    std::vector<std::vector<Point>> all_polygons;
    for (int fi = 0; fi < nf; ++fi) {
        Mesh& fm = fmesh[fi];
        if (fm.is_empty()) continue;
        const auto& face = m_faces[fi];
        // Reversed faces must have their triangle winding flipped so the facet
        // orientation matches the face's outward normal (from_polylines rebuilds
        // vertices from positions, so flipping per-vertex normals here has no effect).
        for (auto& [fk, fverts] : fm.face) {
            std::vector<Point> poly;
            for (auto vi : fverts)
                poly.push_back(fm.vertex.at(vi).position());
            if (face.reversed)
                std::reverse(poly.begin(), poly.end());
            all_polygons.push_back(poly);
        }
    }
    return Mesh::from_polylines(all_polygons, 1e-6);
}// Returns one tessellated Mesh per BRep face, in face order. Vertices are NOT
// shared across faces so face boundaries are hard edges.
std::vector<Mesh> BRep::face_meshes() const {
    return face_meshes_q(false, 0.0, 0.0);
}

// Per-face meshes with an optional tessellation-quality override applied to the
// grid-meshed (direct) faces: when has_quality is true, (max_angle_deg, chord_factor)
// densifies them (and, via the shared-edge coordination, the CDT faces follow).
// When has_quality is false, the default NurbsSurface::mesh() density is used.
std::vector<Mesh> BRep::face_meshes_q(bool has_quality, double max_angle_deg, double chord_factor) const {
    int nf = (int)m_faces.size();

    // Phase 1: classify faces as direct (RemeshNurbsSurfaceGrid) or CDT
    std::vector<bool> face_direct(nf, false);
    for (int fi = 0; fi < nf; ++fi) {
        const auto& face = m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        bool has_inner = false, all_linear = true;
        std::vector<Point> outer_pts;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const auto& loop = m_loops[li];
            if (loop.type == BRepLoopType::Inner) has_inner = true;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                const auto& trim = m_trims[ti];
                if (trim.curve_2d_index < 0 || trim.curve_2d_index >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& crv = m_curves_2d[trim.curve_2d_index];
                if (crv.degree() > 1 || crv.is_rational()) all_linear = false;
                if (loop.type == BRepLoopType::Outer) {
                    if (crv.degree() <= 1 && !crv.is_rational())
                        for (int k = 0; k < crv.cv_count() - 1; ++k) outer_pts.push_back(crv.get_cv(k));
                }
            }
        }
        bool direct = !has_inner && all_linear;
        if (direct && !outer_pts.empty()) {
            auto [u0, u1] = srf.domain(0);
            auto [v0, v1] = srf.domain(1);
            double tol = std::max(u1 - u0, v1 - v0) * 0.01;
            double bb_umin = 1e30, bb_umax = -1e30, bb_vmin = 1e30, bb_vmax = -1e30;
            for (const auto& p : outer_pts) {
                bb_umin = std::min(bb_umin, p[0]); bb_umax = std::max(bb_umax, p[0]);
                bb_vmin = std::min(bb_vmin, p[1]); bb_vmax = std::max(bb_vmax, p[1]);
            }
            if (std::abs(bb_umin - u0) > tol || std::abs(bb_umax - u1) > tol ||
                std::abs(bb_vmin - v0) > tol || std::abs(bb_vmax - v1) > tol)
                direct = false;
        }
        face_direct[fi] = direct;
    }

    // Phase 2: direct faces. Mesh each via the grid mesher, then record the 3D
    // boundary discretisation along every edge shared with a CDT face.
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
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            for (int ti : m_loops[li].trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int eidx = m_trims[ti].edge_index;
                if (eidx < 0 || eidx >= (int)m_topology_edges.size()) continue;
                if (edge_bnd.count(eidx)) continue;
                // Only extract if this edge is shared with a CDT (non-direct) face.
                bool shared = false;
                for (int oti : m_topology_edges[eidx].trim_indices) {
                    if (oti == ti || oti < 0 || oti >= (int)m_trims.size()) continue;
                    int oli = m_trims[oti].loop_index;
                    if (oli < 0 || oli >= (int)m_loops.size()) continue;
                    int ofi = m_loops[oli].face_index;
                    if (ofi >= 0 && ofi < nf && !face_direct[ofi]) { shared = true; break; }
                }
                if (!shared) continue;
                int c2di = m_trims[ti].curve_2d_index;
                if (c2di < 0 || c2di >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& c2d = m_curves_2d[c2di];
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
                    if (at_v0 && std::abs(iv->second - v0) < vtol * 0.1)
                        pts.push_back({iu->second, vd.position()});
                    else if (at_v1 && std::abs(iv->second - v1) < vtol * 0.1)
                        pts.push_back({iu->second, vd.position()});
                    else if (at_u0 && std::abs(iu->second - u0) < utol * 0.1)
                        pts.push_back({iv->second, vd.position()});
                    else if (at_u1 && std::abs(iu->second - u1) < utol * 0.1)
                        pts.push_back({iv->second, vd.position()});
                }
                std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
                if (pts.size() >= 2) {
                    std::vector<Point> bnd;
                    for (auto& [p, pt] : pts) bnd.push_back(pt);
                    edge_bnd[eidx] = bnd;
                }
            }
        }
    }

    // Phase 3: Mesh CDT faces via NurbsSurfaceTrimmed. For edges shared with a direct
    // face, reuse that face's boundary points (projected into this bilinear face's UV).
    for (int fi = 0; fi < nf; ++fi) {
        if (face_direct[fi]) continue;
        const auto& face = m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        // Bilinear 3D->UV projection frame (valid for the planar cap surfaces).
        Point p00 = srf.get_cv(0, 0), p10 = srf.get_cv(1, 0), p01 = srf.get_cv(0, 1);
        double eux = p10[0]-p00[0], euy = p10[1]-p00[1], euz = p10[2]-p00[2];
        double evx = p01[0]-p00[0], evy = p01[1]-p00[1], evz = p01[2]-p00[2];
        double eu2 = eux*eux+euy*euy+euz*euz, ev2 = evx*evx+evy*evy+evz*evz;
        bool can_project = (srf.degree(0) == 1 && srf.degree(1) == 1 && eu2 > 1e-28 && ev2 > 1e-28);

        NurbsSurfaceTrimmed ts;
        ts.m_surface = srf;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const auto& loop = m_loops[li];
            std::vector<Point> loop_pts;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                const auto& trim = m_trims[ti];
                if (trim.type == BRepTrimType::Singular) continue;
                int eidx = trim.edge_index;
                if (can_project && eidx >= 0 && edge_bnd.count(eidx)) {
                    const auto& bnd = edge_bnd[eidx];
                    for (const auto& pt : bnd) {
                        double dx = pt[0]-p00[0], dy = pt[1]-p00[1], dz = pt[2]-p00[2];
                        double u = (dx*eux+dy*euy+dz*euz) / eu2;
                        double v = (dx*evx+dy*evy+dz*evz) / ev2;
                        loop_pts.push_back(Point(u, v, 0));
                    }
                } else {
                    if (trim.curve_2d_index < 0 || trim.curve_2d_index >= (int)m_curves_2d.size()) continue;
                    const NurbsCurve& crv = m_curves_2d[trim.curve_2d_index];
                    if (crv.degree() <= 1 && !crv.is_rational()) {
                        for (int k = 0; k < crv.cv_count() - 1; ++k)
                            loop_pts.push_back(crv.get_cv(k));
                    } else {
                        int n = std::max(crv.cv_count() * 4, 16);
                        auto [pts, params] = crv.divide_by_count(n, true);
                        for (int k = 0; k < (int)pts.size() - 1; ++k)
                            loop_pts.push_back(pts[k]);
                    }
                }
            }
            if (loop_pts.size() >= 3) {
                NurbsCurve loop_crv = NurbsCurve::create(true, 1, loop_pts);
                if (loop.type == BRepLoopType::Outer)
                    ts.m_outer_loop = loop_crv;
                else
                    ts.m_inner_loops.push_back(loop_crv);
            }
        }
        fmesh[fi] = ts.mesh();
    }

    // Apply reversed flag: flip BOTH winding and normals so the shader's gl_FrontFacing
    // derivation agrees with the stored vertex normals.
    for (int fi = 0; fi < nf; ++fi) {
        if (m_faces[fi].reversed) {
            fmesh[fi].flip();
            for (auto& [vk, vd] : fmesh[fi].vertex) {
                auto n = vd.normal();
                if (n) vd.set_normal(-(*n)[0], -(*n)[1], -(*n)[2]);
            }
        }
    }

    return fmesh;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Evaluation
///////////////////////////////////////////////////////////////////////////////////////////

Point BRep::point_at(int face_idx, double u, double v) const {
    if (face_idx < 0 || face_idx >= (int)m_faces.size()) return Point();
    int si = m_faces[face_idx].surface_index;
    if (si < 0 || si >= (int)m_surfaces.size()) return Point();
    return m_surfaces[si].point_at(u, v);
}

Vector BRep::normal_at(int face_idx, double u, double v) const {
    if (face_idx < 0 || face_idx >= (int)m_faces.size()) return Vector();
    int si = m_faces[face_idx].surface_index;
    if (si < 0 || si >= (int)m_surfaces.size()) return Vector();
    Vector n = m_surfaces[si].normal_at(u, v);
    if (m_faces[face_idx].reversed) return Vector(-n[0], -n[1], -n[2]);
    return n;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Transformation
///////////////////////////////////////////////////////////////////////////////////////////

void BRep::transform() {
    for (auto& srf : m_surfaces) {
        srf.xform = xform;
        srf.transform();
    }
    for (auto& crv : m_curves_3d) {
        crv.xform = xform;
        crv.transform();
    }
    for (auto& pt : m_vertices) {
        double x = xform.m[0]*pt[0]+xform.m[1]*pt[1]+xform.m[2]*pt[2]+xform.m[3];
        double y = xform.m[4]*pt[0]+xform.m[5]*pt[1]+xform.m[6]*pt[2]+xform.m[7];
        double z = xform.m[8]*pt[0]+xform.m[9]*pt[1]+xform.m[10]*pt[2]+xform.m[11];
        pt = Point(x, y, z);
    }
    xform = Xform::identity();
}

BRep BRep::transformed() const {
    BRep b = *this;
    b.transform();
    return b;
}

///////////////////////////////////////////////////////////////////////////////////////////
// JSON Serialization
///////////////////////////////////////////////////////////////////////////////////////////

static nlohmann::ordered_json trim_type_to_json(BRepTrimType t) {
    switch (t) {
        case BRepTrimType::Boundary: return "boundary";
        case BRepTrimType::Mated: return "mated";
        case BRepTrimType::Seam: return "seam";
        case BRepTrimType::Singular: return "singular";
    }
    return "boundary";
}

static BRepTrimType trim_type_from_json(const std::string& s) {
    if (s == "mated") return BRepTrimType::Mated;
    if (s == "seam") return BRepTrimType::Seam;
    if (s == "singular") return BRepTrimType::Singular;
    return BRepTrimType::Boundary;
}

static nlohmann::ordered_json loop_type_to_json(BRepLoopType t) {
    return (t == BRepLoopType::Inner) ? "inner" : "outer";
}

static BRepLoopType loop_type_from_json(const std::string& s) {
    return (s == "inner") ? BRepLoopType::Inner : BRepLoopType::Outer;
}

nlohmann::ordered_json BRep::jsondump() const {
    nlohmann::ordered_json j;
    j["curves_2d"] = nlohmann::ordered_json::array();
    for (const auto& c : m_curves_2d) j["curves_2d"].push_back(c.jsondump());
    j["curves_3d"] = nlohmann::ordered_json::array();
    for (const auto& c : m_curves_3d) j["curves_3d"].push_back(c.jsondump());
    j["faces"] = nlohmann::ordered_json::array();
    for (const auto& f : m_faces) {
        nlohmann::ordered_json fj;
        if (f.facecolor.a > 0) fj["facecolor"] = f.facecolor.jsondump();
        fj["loop_indices"] = f.loop_indices;
        fj["reversed"] = f.reversed;
        fj["surface_index"] = f.surface_index;
        j["faces"].push_back(fj);
    }
    j["guid"] = guid();
    j["loops"] = nlohmann::ordered_json::array();
    for (const auto& l : m_loops) {
        nlohmann::ordered_json lj;
        lj["face_index"] = l.face_index;
        lj["trim_indices"] = l.trim_indices;
        lj["type"] = loop_type_to_json(l.type);
        j["loops"].push_back(lj);
    }
    j["name"] = name;
    j["surfaces"] = nlohmann::ordered_json::array();
    for (const auto& s : m_surfaces) j["surfaces"].push_back(s.jsondump());
    j["surfacecolor"] = surfacecolor.jsondump();
    j["topology_edges"] = nlohmann::ordered_json::array();
    for (const auto& e : m_topology_edges) {
        nlohmann::ordered_json ej;
        ej["curve_3d_index"] = e.curve_3d_index;
        ej["end_vertex"] = e.end_vertex;
        ej["start_vertex"] = e.start_vertex;
        ej["trim_indices"] = e.trim_indices;
        j["topology_edges"].push_back(ej);
    }
    j["topology_vertices"] = nlohmann::ordered_json::array();
    for (const auto& v : m_topology_vertices) {
        nlohmann::ordered_json vj;
        vj["edge_indices"] = v.edge_indices;
        vj["point_index"] = v.point_index;
        j["topology_vertices"].push_back(vj);
    }
    j["trims"] = nlohmann::ordered_json::array();
    for (const auto& t : m_trims) {
        nlohmann::ordered_json tj;
        tj["curve_2d_index"] = t.curve_2d_index;
        tj["edge_index"] = t.edge_index;
        tj["loop_index"] = t.loop_index;
        tj["reversed"] = t.reversed;
        tj["type"] = trim_type_to_json(t.type);
        j["trims"].push_back(tj);
    }
    j["type"] = "BRep";
    j["vertices"] = nlohmann::ordered_json::array();
    for (const auto& v : m_vertices)
        j["vertices"].push_back(nlohmann::ordered_json::array({v[0], v[1], v[2]}));
    j["width"] = width;
    j["xform"] = xform.jsondump();
    return j;
}

BRep BRep::jsonload(const nlohmann::json& data) {
    BRep b;
    if (data.contains("guid")) b.guid() = data["guid"];
    if (data.contains("name")) b.name = data["name"];
    if (data.contains("width")) b.width = data["width"];
    if (data.contains("surfacecolor")) b.surfacecolor = Color::jsonload(data["surfacecolor"]);
    if (data.contains("xform")) b.xform = Xform::jsonload(data["xform"]);
    if (data.contains("curves_2d"))
        for (const auto& c : data["curves_2d"]) b.m_curves_2d.push_back(NurbsCurve::jsonload(c));
    if (data.contains("curves_3d"))
        for (const auto& c : data["curves_3d"]) b.m_curves_3d.push_back(NurbsCurve::jsonload(c));
    if (data.contains("surfaces"))
        for (const auto& s : data["surfaces"]) b.m_surfaces.push_back(NurbsSurface::jsonload(s));
    if (data.contains("vertices"))
        for (const auto& v : data["vertices"]) b.m_vertices.push_back(Point(v[0], v[1], v[2]));
    if (data.contains("topology_vertices"))
        for (const auto& v : data["topology_vertices"]) {
            BRepVertex tv;
            tv.point_index = v["point_index"];
            tv.edge_indices = v["edge_indices"].get<std::vector<int>>();
            b.m_topology_vertices.push_back(tv);
        }
    if (data.contains("topology_edges"))
        for (const auto& e : data["topology_edges"]) {
            BRepEdge te;
            te.curve_3d_index = e["curve_3d_index"];
            te.start_vertex = e["start_vertex"];
            te.end_vertex = e["end_vertex"];
            te.trim_indices = e["trim_indices"].get<std::vector<int>>();
            b.m_topology_edges.push_back(te);
        }
    if (data.contains("trims"))
        for (const auto& t : data["trims"]) {
            BRepTrim bt;
            bt.curve_2d_index = t["curve_2d_index"];
            bt.edge_index = t["edge_index"];
            bt.loop_index = t["loop_index"];
            bt.reversed = t["reversed"];
            bt.type = trim_type_from_json(t["type"]);
            b.m_trims.push_back(bt);
        }
    if (data.contains("loops"))
        for (const auto& l : data["loops"]) {
            BRepLoop bl;
            bl.face_index = l["face_index"];
            bl.trim_indices = l["trim_indices"].get<std::vector<int>>();
            bl.type = loop_type_from_json(l["type"]);
            b.m_loops.push_back(bl);
        }
    if (data.contains("faces"))
        for (const auto& f : data["faces"]) {
            BRepFace bf;
            bf.surface_index = f["surface_index"];
            bf.loop_indices = f["loop_indices"].get<std::vector<int>>();
            bf.reversed = f["reversed"];
            if (f.contains("facecolor")) bf.facecolor = Color::jsonload(f["facecolor"]);
            b.m_faces.push_back(bf);
        }
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

std::string BRep::pb_dumps() const {
    session_proto::BRep proto;
    proto.set_guid(guid());
    proto.set_name(name);
    proto.set_width(width);

    for (const auto& c : m_curves_2d) {
        auto* p = proto.add_curves_2d();
        p->ParseFromString(c.pb_dumps());
    }
    for (const auto& c : m_curves_3d) {
        auto* p = proto.add_curves_3d();
        p->ParseFromString(c.pb_dumps());
    }
    for (const auto& s : m_surfaces) {
        auto* p = proto.add_surfaces();
        p->ParseFromString(s.pb_dumps());
    }
    for (const auto& v : m_vertices) {
        auto* p = proto.add_vertices();
        p->set_x(v[0]); p->set_y(v[1]); p->set_z(v[2]);
    }
    for (const auto& tv : m_topology_vertices) {
        auto* p = proto.add_topology_vertices();
        p->set_point_index(tv.point_index);
        for (int ei : tv.edge_indices) p->add_edge_indices(ei);
    }
    for (const auto& te : m_topology_edges) {
        auto* p = proto.add_topology_edges();
        p->set_curve_3d_index(te.curve_3d_index);
        p->set_start_vertex(te.start_vertex);
        p->set_end_vertex(te.end_vertex);
        for (int ti : te.trim_indices) p->add_trim_indices(ti);
    }
    for (const auto& t : m_trims) {
        auto* p = proto.add_trims();
        p->set_curve_2d_index(t.curve_2d_index);
        p->set_edge_index(t.edge_index);
        p->set_loop_index(t.loop_index);
        p->set_reversed(t.reversed);
        p->set_type(static_cast<session_proto::BRepTrimType>(t.type));
    }
    for (const auto& l : m_loops) {
        auto* p = proto.add_loops();
        for (int ti : l.trim_indices) p->add_trim_indices(ti);
        p->set_face_index(l.face_index);
        p->set_type(static_cast<session_proto::BRepLoopType>(l.type));
    }
    for (const auto& f : m_faces) {
        auto* p = proto.add_faces();
        p->set_surface_index(f.surface_index);
        for (int li : f.loop_indices) p->add_loop_indices(li);
        p->set_reversed(f.reversed);
        if (f.facecolor.a > 0) {
            auto* fc = p->mutable_facecolor();
            fc->set_r(f.facecolor.r); fc->set_g(f.facecolor.g);
            fc->set_b(f.facecolor.b); fc->set_a(f.facecolor.a);
        }
    }

    auto* color_proto = proto.mutable_surfacecolor();
    color_proto->set_name(surfacecolor.name);
    color_proto->set_r(surfacecolor.r);
    color_proto->set_g(surfacecolor.g);
    color_proto->set_b(surfacecolor.b);
    color_proto->set_a(surfacecolor.a);

    auto* xform_proto = proto.mutable_xform();
    xform_proto->set_guid(xform.guid());
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i) xform_proto->add_matrix(xform.m[i]);

    return proto.SerializeAsString();
}

BRep BRep::pb_loads(const std::string& data) {
    session_proto::BRep proto;
    proto.ParseFromString(data);
    BRep b;
    b.guid() = proto.guid();
    b.name = proto.name();
    b.width = proto.width();

    for (int i = 0; i < proto.curves_2d_size(); ++i)
        b.m_curves_2d.push_back(NurbsCurve::pb_loads(proto.curves_2d(i).SerializeAsString()));
    for (int i = 0; i < proto.curves_3d_size(); ++i)
        b.m_curves_3d.push_back(NurbsCurve::pb_loads(proto.curves_3d(i).SerializeAsString()));
    for (int i = 0; i < proto.surfaces_size(); ++i)
        b.m_surfaces.push_back(NurbsSurface::pb_loads(proto.surfaces(i).SerializeAsString()));
    for (int i = 0; i < proto.vertices_size(); ++i) {
        const auto& v = proto.vertices(i);
        b.m_vertices.push_back(Point(v.x(), v.y(), v.z()));
    }
    for (int i = 0; i < proto.topology_vertices_size(); ++i) {
        const auto& tv = proto.topology_vertices(i);
        BRepVertex bv;
        bv.point_index = tv.point_index();
        for (int j = 0; j < tv.edge_indices_size(); ++j)
            bv.edge_indices.push_back(tv.edge_indices(j));
        b.m_topology_vertices.push_back(bv);
    }
    for (int i = 0; i < proto.topology_edges_size(); ++i) {
        const auto& te = proto.topology_edges(i);
        BRepEdge be;
        be.curve_3d_index = te.curve_3d_index();
        be.start_vertex = te.start_vertex();
        be.end_vertex = te.end_vertex();
        for (int j = 0; j < te.trim_indices_size(); ++j)
            be.trim_indices.push_back(te.trim_indices(j));
        b.m_topology_edges.push_back(be);
    }
    for (int i = 0; i < proto.trims_size(); ++i) {
        const auto& t = proto.trims(i);
        BRepTrim bt;
        bt.curve_2d_index = t.curve_2d_index();
        bt.edge_index = t.edge_index();
        bt.loop_index = t.loop_index();
        bt.reversed = t.reversed();
        bt.type = static_cast<BRepTrimType>(t.type());
        b.m_trims.push_back(bt);
    }
    for (int i = 0; i < proto.loops_size(); ++i) {
        const auto& l = proto.loops(i);
        BRepLoop bl;
        bl.face_index = l.face_index();
        for (int j = 0; j < l.trim_indices_size(); ++j)
            bl.trim_indices.push_back(l.trim_indices(j));
        bl.type = static_cast<BRepLoopType>(l.type());
        b.m_loops.push_back(bl);
    }
    for (int i = 0; i < proto.faces_size(); ++i) {
        const auto& f = proto.faces(i);
        BRepFace bf;
        bf.surface_index = f.surface_index();
        for (int j = 0; j < f.loop_indices_size(); ++j)
            bf.loop_indices.push_back(f.loop_indices(j));
        bf.reversed = f.reversed();
        if (f.has_facecolor())
            bf.facecolor = Color(f.facecolor().r(), f.facecolor().g(), f.facecolor().b(), f.facecolor().a());
        b.m_faces.push_back(bf);
    }

    const auto& cp = proto.surfacecolor();
    b.surfacecolor.name = cp.name();
    b.surfacecolor.r = cp.r();
    b.surfacecolor.g = cp.g();
    b.surfacecolor.b = cp.b();
    b.surfacecolor.a = cp.a();

    const auto& xp = proto.xform();
    b.xform.guid() = xp.guid();
    b.xform.name = xp.name();
    for (int i = 0; i < 16 && i < xp.matrix_size(); ++i)
        b.xform.m[i] = xp.matrix(i);

    return b;
}

void BRep::pb_dump(const std::string& filename) const {
    std::string data = pb_dumps();
    std::ofstream file(filename, std::ios::binary);
    file.write(data.data(), data.size());
}

BRep BRep::pb_load(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    std::string data((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
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
