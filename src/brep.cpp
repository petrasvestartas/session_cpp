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
        if (std::getenv("SESSION_SOLID_DBG")) {
            Point ps(0,0,0), pe(0,0,0);
            if (ci >= 0 && ci < (int)m_curves_3d.size()) {
                const NurbsCurve& c = m_curves_3d[ci];
                auto dc = c.domain();
                ps = c.point_at(dc.first); pe = c.point_at(dc.second);
            }
            std::fprintf(stderr, "[SOLID] e=%d nt=%d s(%.4f,%.4f,%.4f) e(%.4f,%.4f,%.4f)\n",
                (int)(&e - &m_topology_edges[0]), (int)e.trim_indices.size(),
                ps[0],ps[1],ps[2], pe[0],pe[1],pe[2]);
            continue;
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

namespace { bool cylinder_of_surface(const NurbsSurface& s, Point& A, Vector& W, double& R); }

int BRep::check_trim_orientation(bool verbose) const {
    int violations = 0;
    for (int fi = 0; fi < (int)m_faces.size(); ++fi) {
        const BRepFace& face = m_faces[fi];
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];
        auto du = srf.domain(0); auto dv = srf.domain(1);
        double ru = du.second - du.first, rv = dv.second - dv.first;
        double tol_uv = 1e-6 * std::max(ru, rv);
        double jump_uv = 0.1 * std::max(ru, rv);
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const BRepLoop& loop = m_loops[li];
            bool have_prev = false, jumpy = false;
            Point first_pt(0,0,0), prev_pt(0,0,0);
            double area2 = 0.0;   // 2*signed area, shoelace over trim endpoints + samples
            Point area_prev(0,0,0); bool area_first = true;
            int ntrim = 0;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                int c2 = m_trims[ti].curve_2d_index;
                if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                const NurbsCurve& pc = m_curves_2d[c2];
                if (!pc.is_valid()) continue;
                auto dc = pc.domain();
                bool rev = m_trims[ti].reversed;
                Point s = pc.point_at(rev ? dc.second : dc.first);
                Point e = pc.point_at(rev ? dc.first : dc.second);
                if (have_prev) {
                    double gap = std::hypot(s[0]-prev_pt[0], s[1]-prev_pt[1]);
                    if (gap > jump_uv) jumpy = true;
                    else if (gap > tol_uv) {
                        ++violations;
                        if (verbose) std::fprintf(stderr,
                            "[ORIENT] f=%d loop=%d trim=%d chain gap %.3e at uv(%.4f,%.4f)\n",
                            fi, li, ti, gap, s[0], s[1]);
                    }
                } else { first_pt = s; }
                const int NS = 8;
                for (int k = 0; k <= NS; ++k) {
                    double f = (double)k / NS;
                    double t = rev ? dc.second - (dc.second-dc.first)*f
                                   : dc.first + (dc.second-dc.first)*f;
                    Point p = pc.point_at(t);
                    if (!area_first) area2 += area_prev[0]*p[1] - p[0]*area_prev[1];
                    area_prev = p; area_first = false;
                }
                prev_pt = e; have_prev = true; ++ntrim;
            }
            if (!have_prev || ntrim == 0) continue;
            double close_gap = std::hypot(first_pt[0]-prev_pt[0], first_pt[1]-prev_pt[1]);
            if (close_gap > jump_uv) jumpy = true;
            else if (close_gap > tol_uv) {
                ++violations;
                if (verbose) std::fprintf(stderr, "[ORIENT] f=%d loop=%d closure gap %.3e\n", fi, li, close_gap);
            }
            area2 += area_prev[0]*first_pt[1] - first_pt[0]*area_prev[1];
            if (!jumpy && std::abs(area2) > tol_uv * std::max(ru, rv)) {
                bool ccw = area2 > 0.0;
                bool want_ccw = (loop.type == BRepLoopType::Outer);
                if (ccw != want_ccw) {
                    ++violations;
                    if (verbose) std::fprintf(stderr,
                        "[ORIENT] f=%d loop=%d type=%d area2=%.3e wrong sense (ntrim=%d)\n",
                        fi, li, (int)loop.type, area2, ntrim);
                }
            } else if (jumpy && verbose) {
                std::fprintf(stderr, "[ORIENT] f=%d loop=%d SEAM-JUMP loop (ntrim=%d) - sense unchecked\n", fi, li, ntrim);
            }
        }
    }
    return violations;
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
            // Integration cells ALIGNED with the pcurve's knot spans: a uniform NS split straddles
            // knots, putting an only-C^{deg-1} integrand under a smooth Gauss rule -- aliasing
            // error that MOVES when a trim is split (the split box x tor spiric discs shifted the
            // volume by 2e-6). Per-span Gauss-5 is EXACT for a non-rational pcurve on an affine
            // patch (polynomial integrand, degree <= 2*deg-1 <= 9); long smooth spans (exact
            // rational arcs, 1-3 spans) keep >= NS cells via per-span subdivision.
            std::vector<double> cells;
            {
                std::vector<double> ks;
                ks.push_back(sg.t0);
                for (int k = 0; k < sg.pc->nurbsknot_count(); ++k) {
                    double t = sg.pc->nurbsknot(k);
                    if (t > sg.t0 + 1e-14 && t < sg.t1 - 1e-14 && t - ks.back() > 1e-14)
                        ks.push_back(t);
                }
                ks.push_back(sg.t1);
                int nspan = (int)ks.size() - 1;
                int sub = std::max(1, (NS + nspan - 1) / nspan);
                for (int i = 0; i < nspan; ++i)
                    for (int q = 0; q < sub; ++q)
                        cells.push_back(ks[i] + (ks[i+1]-ks[i])*q/sub);
                cells.push_back(sg.t1);
            }
            for (int s = 0; s + 1 < (int)cells.size(); s++) {
                double a = cells[s], b = cells[s+1];
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
    auto face_interior = [&](const BRepFace& face, const NurbsSurface& srf, Point& P3, Vector& Nnat,
                             double& ocu, double& ocv) {
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
                // Cap: a dense polyline pcurve (torus pullback of a sampled section loop,
                // cv ~ 4000) does not need cv*3 samples to seed the CDT -- the uncapped
                // count produced ~50k-vertex polygons and a quadratic CDT.
                int n = std::min(std::max(pc.cv_count()*3, 6), 1024);
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
        ocu = cu; ocv = cv;
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

    // ---- PASS 0: shell-consistent face signs by ORIENTATION PROPAGATION (OCCT shell healing).
    // Per-face parity votes are heuristics only (tessellation cracks near tangencies break ray
    // parity; a quadric outward reference inverts on cavity shells). The boolean's validated
    // invariant -- mated trims traverse their shared edge ANTI-PARALLEL exactly when the two
    // faces' natural normals are in the same orientation class -- makes RELATIVE signs exact:
    // propagate them across every 2-trim edge, then fix each connected shell's single global
    // flip by weighted evidence (planar supporting-plane tests are exact geometry -> weight 3;
    // double-sided parity probes that disagree between the two sides -> weight 1). A face sign
    // can then only be wrong if the evidence outvotes the WHOLE shell, and disjoint lobes /
    // cavity shells are seeded independently.
    int nfc = (int)m_faces.size();
    std::vector<Point>  fP3(nfc, Point(0,0,0));
    std::vector<Vector> fNn(nfc, Vector(0,0,0));
    std::vector<double> fCU(nfc, 0.0), fCVv(nfc, 0.0);
    std::vector<double> fsign(nfc, 1.0);
    {
        std::vector<char> fvalid(nfc, 0);
        for (int fi = 0; fi < nfc; ++fi) {
            const BRepFace& fc = m_faces[fi];
            if (fc.surface_index < 0 || fc.surface_index >= (int)m_surfaces.size()) continue;
            face_interior(fc, m_surfaces[fc.surface_index], fP3[fi], fNn[fi], fCU[fi], fCVv[fi]);
            if (fNn[fi].magnitude() >= 1e-12) fvalid[fi] = 1;
        }
        auto face_of_trim = [&](int ti) -> int {
            if (ti < 0 || ti >= (int)m_trims.size()) return -1;
            int li = m_trims[ti].loop_index;
            if (li < 0 || li >= (int)m_loops.size()) return -1;
            return m_loops[li].face_index;
        };
        // Per-face UV region polygons (light sampling) for geometric material-side tests.
        std::vector<char> fpoly_built(nfc, 0);
        std::vector<std::vector<std::vector<std::array<double,2>>>> fpoly_out(nfc), fpoly_in(nfc);
        auto build_fpolys = [&](int fi) {
            if (fpoly_built[fi]) return;
            fpoly_built[fi] = 1;
            for (int li : m_faces[fi].loop_indices) {
                if (li < 0 || li >= (int)m_loops.size()) continue;
                std::vector<std::array<double,2>> poly;
                for (int ti : m_loops[li].trim_indices) {
                    if (ti < 0 || ti >= (int)m_trims.size()) continue;
                    int c2 = m_trims[ti].curve_2d_index;
                    if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                    const NurbsCurve& pc = m_curves_2d[c2];
                    if (!pc.is_valid()) continue;
                    auto dc = pc.domain();
                    int n = std::min(std::max(pc.cv_count()*2, 8), 256);
                    for (int k = 0; k < n; ++k) {
                        Point uv = pc.point_at(dc.first + (dc.second-dc.first)*k/n);
                        poly.push_back({uv[0], uv[1]});
                    }
                }
                if (poly.size() < 3) continue;
                if (m_loops[li].type == BRepLoopType::Outer) fpoly_out[fi].push_back(std::move(poly));
                else fpoly_in[fi].push_back(std::move(poly));
            }
        };
        auto in_face_uv = [&](int fi, double u, double v) -> bool {
            build_fpolys(fi);
            auto pip = [&](const std::vector<std::array<double,2>>& p) {
                bool inside = false;
                for (size_t i = 0, j = p.size()-1; i < p.size(); j = i++) {
                    if ((p[i][1] > v) != (p[j][1] > v) &&
                        u < (p[j][0]-p[i][0])*(v-p[i][1])/(p[j][1]-p[i][1]) + p[i][0])
                        inside = !inside;
                }
                return inside;
            };
            bool ok = fpoly_out[fi].empty();
            for (auto& op : fpoly_out[fi]) if (pip(op)) { ok = true; break; }
            if (!ok) return false;
            for (auto& ip : fpoly_in[fi]) if (pip(ip)) return false;
            return true;
        };
        // 3D traversal direction of a trim near its pcurve middle, NORMALIZED to material-left:
        // stored direction conventions are not reliable across construction paths (the box x sph
        // seam-straddling half-caps store material-RIGHT trims), so the material side is MEASURED
        // in UV (probe left/right of the traversal against the face's own region) and the
        // direction flipped when material sits right. After normalization, anti-parallel mates
        // <=> same natural-orientation class is a theorem, not a convention.
        auto trim_dir = [&](int ti) -> Vector {
            const BRepTrim& T = m_trims[ti];
            int c2 = T.curve_2d_index; int fi = face_of_trim(ti);
            if (c2 < 0 || c2 >= (int)m_curves_2d.size() || fi < 0) return Vector(0,0,0);
            int si = m_faces[fi].surface_index;
            if (si < 0 || si >= (int)m_surfaces.size()) return Vector(0,0,0);
            const NurbsCurve& pc = m_curves_2d[c2];
            if (!pc.is_valid()) return Vector(0,0,0);
            auto dc = pc.domain();
            double tm = 0.5*(dc.first+dc.second), dt = (dc.second-dc.first)*1e-3;
            Point a = pc.point_at(tm - dt), b = pc.point_at(tm + dt);
            if (T.reversed) std::swap(a, b);              // traversal order
            Point A = m_surfaces[si].point_at(a[0], a[1]);
            Point B = m_surfaces[si].point_at(b[0], b[1]);
            Vector d(B[0]-A[0], B[1]-A[1], B[2]-A[2]);
            double tx = b[0]-a[0], ty = b[1]-a[1];
            double tn = std::sqrt(tx*tx+ty*ty);
            if (tn < 1e-15) return Vector(0,0,0);
            auto [su0,su1] = m_surfaces[si].domain(0);
            auto [sv0,sv1] = m_surfaces[si].domain(1);
            Point m = pc.point_at(tm);
            for (double f : {2e-3, 8e-3}) {
                double eu = (su1-su0)*f, ev = (sv1-sv0)*f;
                double lu = m[0] - ty/tn*eu, lv = m[1] + tx/tn*ev;   // left of traversal
                double ru = m[0] + ty/tn*eu, rv = m[1] - tx/tn*ev;   // right of traversal
                bool left_in = in_face_uv(fi, lu, lv);
                bool right_in = in_face_uv(fi, ru, rv);
                if (left_in == right_in) continue;                    // probe straddles another edge
                if (!left_in) return Vector(-d[0], -d[1], -d[2]);     // material-right: normalize
                return d;
            }
            return Vector(0,0,0);                                     // ambiguous: skip this trim
        };
        std::vector<std::vector<std::pair<int,int>>> adj(nfc);
        for (const auto& E : m_topology_edges) {
            if ((int)E.trim_indices.size() != 2) continue;
            int t1 = E.trim_indices[0], t2 = E.trim_indices[1];
            int f1 = face_of_trim(t1), f2 = face_of_trim(t2);
            if (f1 < 0 || f2 < 0 || f1 == f2) continue;
            if (!fvalid[f1] || !fvalid[f2]) continue;
            Vector d1 = trim_dir(t1), d2 = trim_dir(t2);
            double m1 = d1.magnitude(), m2 = d2.magnitude();
            if (m1 < 1e-12 || m2 < 1e-12) continue;
            double dp = (d1[0]*d2[0]+d1[1]*d2[1]+d1[2]*d2[2]) / (m1*m2);
            if (std::abs(dp) < 0.5) continue;             // ambiguous tangents: skip this edge
            int rel = dp < 0 ? +1 : -1;                   // anti-parallel = same orientation class
            if (std::getenv("SESSION_SIGN_DBG"))
                std::fprintf(stderr, "[ADJ] e=%d f%d<->f%d rel=%+d dp=%+.3f t(%d,%d)\n",
                    (int)(&E - &m_topology_edges[0]), f1, f2, rel, dp, t1, t2);
            adj[f1].push_back({f2, rel});
            adj[f2].push_back({f1, rel});
        }
        std::vector<int> comp(nfc, -1), relsgn(nfc, 1);
        int ncomp = 0;
        for (int s0 = 0; s0 < nfc; ++s0) {
            if (comp[s0] >= 0 || !fvalid[s0]) continue;
            comp[s0] = ncomp; relsgn[s0] = 1;
            std::vector<int> stk{s0};
            while (!stk.empty()) {
                int f = stk.back(); stk.pop_back();
                for (auto& [g, rel] : adj[f]) {
                    int want = rel > 0 ? relsgn[f] : -relsgn[f];
                    if (comp[g] < 0) { comp[g] = ncomp; relsgn[g] = want; stk.push_back(g); }
                }
            }
            ++ncomp;
        }
        const double sdirs[3][3] = {{0.5773502691, 0.6539124, 0.5023147},
                                    {-0.6172133998, 0.5330976, 0.5788126},
                                    {0.4419417382, -0.5303300859, 0.7237468644}};
        auto parity_in = [&](const Point& q) {
            int votes = 0; const double big = 1e6;
            for (int d = 0; d < 3; ++d) {
                Line ray(q[0], q[1], q[2], q[0]+sdirs[d][0]*big, q[1]+sdirs[d][1]*big, q[2]+sdirs[d][2]*big);
                auto hits = Intersection::ray_mesh(ray, bmesh, 1e-9, true);
                if ((hits.size() % 2) == 1) ++votes;
            }
            return votes >= 2;
        };
        std::vector<double> score(std::max(ncomp, 1), 0.0);
        for (int fi = 0; fi < nfc; ++fi) {
            if (!fvalid[fi]) continue;
            const NurbsSurface& srf = m_surfaces[m_faces[fi].surface_index];
            double ev = 0.0, wt = 0.0;
            if (is_planar(srf)) {
                // Supporting-plane outward sign: when the whole solid lies on one side of this
                // face's plane, the outward normal is the empty side -- exact and ray-free.
                double dmax = -1e300, dmin = 1e300;
                for (const auto& vq : m_vertices) {
                    double d = (vq[0]-fP3[fi][0])*fNn[fi][0] + (vq[1]-fP3[fi][1])*fNn[fi][1]
                             + (vq[2]-fP3[fi][2])*fNn[fi][2];
                    dmax = std::max(dmax, d); dmin = std::min(dmin, d);
                }
                double tol_sup = (diag > 0 ? diag : 1.0) * 5e-3;
                if (dmax <= tol_sup)       { ev = +1.0; wt = 3.0; }
                else if (dmin >= -tol_sup) { ev = -1.0; wt = 3.0; }
            }
            if (wt == 0.0) {
                Point pp(fP3[fi][0]+eps*fNn[fi][0], fP3[fi][1]+eps*fNn[fi][1], fP3[fi][2]+eps*fNn[fi][2]);
                Point pm(fP3[fi][0]-eps*fNn[fi][0], fP3[fi][1]-eps*fNn[fi][1], fP3[fi][2]-eps*fNn[fi][2]);
                bool in_p = parity_in(pp), in_m = parity_in(pm);
                if (in_p != in_m) { ev = in_p ? -1.0 : +1.0; wt = 1.0; }
                // inconsistent double probe (one side's rays graze bad triangles, e.g. the
                // inside-cavity probe next to sphere pole fans): keep the OUTWARD-side parity
                // as weak evidence rather than discarding the face's vote entirely -- a lone
                // isolated shell (box-with-cavity's sphere) has no other voters.
                else { ev = in_p ? -1.0 : +1.0; wt = 0.5; }
            }
            if (wt > 0.0) score[comp[fi]] += wt * ev * relsgn[fi];
        }
        for (int fi = 0; fi < nfc; ++fi) {
            if (!fvalid[fi]) continue;
            double flip = score[comp[fi]] < 0 ? -1.0 : 1.0;
            fsign[fi] = relsgn[fi] * flip;
            if (std::getenv("SESSION_SIGN_DBG"))
                std::fprintf(stderr, "[SIGN0] f=%d comp=%d rel=%+d score=%+.1f sign=%+.0f P3=(%.4f,%.4f,%.4f)\n",
                             fi, comp[fi], relsgn[fi], score[comp[fi]], fsign[fi],
                             fP3[fi][0], fP3[fi][1], fP3[fi][2]);
        }
    }

    double total = 0.0;
    for (const auto& face : m_faces) {
        if (face.surface_index < 0 || face.surface_index >= (int)m_surfaces.size()) continue;
        const NurbsSurface& srf = m_surfaces[face.surface_index];

        int fidx = (int)(&face - &m_faces[0]);
        const Point& P3 = fP3[fidx];
        const Vector& Nnat = fNn[fidx];
        if (Nnat.magnitude() < 1e-12) continue;
        double sign = fsign[fidx];

        // Quadric recognition (sphere/cylinder/torus), hoisted BEFORE the masked Gauss: when the
        // analytic boundary-integral flux below overrides the Gauss result (recognized kind and a
        // non-rect trim), the 384^2 masked Gauss is dead weight and is skipped entirely.
        int q_kind = 0;                     // 1=sphere, 2=cylinder, 3=torus
        Point q_C(0,0,0); Vector q_Zs(0,0,1), q_Xs(1,0,0), q_Ys(0,1,0);
        double q_R = 0, q_r2 = 0;           // sphere/cyl radius R; torus major R minor r2
        if (!is_planar(srf)) {
            auto [su0,su1]=srf.domain(0); auto [sv0,sv1]=srf.domain(1);
            auto dot3=[](const Vector&a,const Vector&b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];};
            double um=0.5*(su0+su1);
            Point Ps=srf.point_at(um,sv0), Pn=srf.point_at(um,sv1), Pm=srf.point_at(um,0.5*(sv0+sv1));
            Vector axis(Pn[0]-Ps[0],Pn[1]-Ps[1],Pn[2]-Ps[2]);
            double Rt = 0.5*axis.magnitude();
            if (Rt > 1e-9) {
                Point Ct(0.5*(Ps[0]+Pn[0]),0.5*(Ps[1]+Pn[1]),0.5*(Ps[2]+Pn[2]));
                Vector Zt(axis[0]/(2*Rt),axis[1]/(2*Rt),axis[2]/(2*Rt));
                bool is_sphere=true;
                for(int i=0;i<=3&&is_sphere;++i)for(int j=0;j<=3&&is_sphere;++j){
                    Point p=srf.point_at(su0+(su1-su0)*i/3.0, sv0+(sv1-sv0)*j/3.0);
                    if (std::abs(p.distance(Ct)-Rt) > Rt*1e-4 + 1e-6) is_sphere=false;
                }
                if (is_sphere) {
                    Vector dm(Pm[0]-Ct[0],Pm[1]-Ct[1],Pm[2]-Ct[2]);
                    double dz=dm[0]*Zt[0]+dm[1]*Zt[1]+dm[2]*Zt[2];
                    Vector Xt(dm[0]-dz*Zt[0],dm[1]-dz*Zt[1],dm[2]-dz*Zt[2]);
                    double xn=Xt.magnitude();
                    if (xn > 1e-12) {
                        q_kind=1; q_C=Ct; q_Zs=Zt; q_R=Rt;
                        q_Xs=Vector(Xt[0]/xn,Xt[1]/xn,Xt[2]/xn); q_Ys=cross(q_Zs,q_Xs);
                    }
                }
            }
            if (!q_kind) {
                Point Ax; Vector Wc; double Rc = 0;
                if (cylinder_of_surface(srf, Ax, Wc, Rc)) {
                    Point Q0 = srf.point_at(su0, 0.5*(sv0+sv1));
                    Vector d0(Q0[0]-Ax[0], Q0[1]-Ax[1], Q0[2]-Ax[2]);
                    double dz0 = dot3(d0, Wc);
                    Vector Xc(d0[0]-dz0*Wc[0], d0[1]-dz0*Wc[1], d0[2]-dz0*Wc[2]);
                    double xl = Xc.magnitude();
                    if (xl > 1e-12) {
                        q_kind=2; q_C=Ax; q_Zs=Wc; q_R=Rc;
                        q_Xs=Vector(Xc[0]/xl,Xc[1]/xl,Xc[2]/xl); q_Ys=cross(q_Zs,q_Xs);
                    }
                }
            }
            if (!q_kind) {
                // torus: tube-circle centres at u0 and its antipode give the centre; a third
                // centre fixes the axis; verified on a grid against (rho-R, z) tube distance.
                double vm2 = 0.5*(sv0+sv1);
                auto tube_c = [&](double u){ Point a=srf.point_at(u,sv0), b=srf.point_at(u,vm2);
                    return Point(0.5*(a[0]+b[0]),0.5*(a[1]+b[1]),0.5*(a[2]+b[2])); };
                Point c1=tube_c(su0), c2=tube_c(0.5*(su0+su1)), c3p=tube_c(su0+0.25*(su1-su0));
                Point Ct(0.5*(c1[0]+c2[0]),0.5*(c1[1]+c2[1]),0.5*(c1[2]+c2[2]));
                Vector a1(c1[0]-Ct[0],c1[1]-Ct[1],c1[2]-Ct[2]);
                Vector a3(c3p[0]-Ct[0],c3p[1]-Ct[1],c3p[2]-Ct[2]);
                Vector Zt = cross(a1, a3);
                double zl = Zt.magnitude(), Rmaj = a1.magnitude();
                if (zl > 1e-12 && Rmaj > 1e-9) {
                    Zt = Vector(Zt[0]/zl, Zt[1]/zl, Zt[2]/zl);
                    double rmin = srf.point_at(su0,sv0).distance(c1);
                    bool is_torus = rmin > 1e-9 && rmin < Rmaj*(1.0-1e-6);
                    for(int i=0;i<=3&&is_torus;++i)for(int j=0;j<=2&&is_torus;++j){
                        Point p=srf.point_at(su0+(su1-su0)*i/3.0, sv0+(sv1-sv0)*j/2.0);
                        Vector w(p[0]-Ct[0],p[1]-Ct[1],p[2]-Ct[2]);
                        double z=dot3(w,Zt);
                        double rho=std::sqrt(std::max(0.0, dot3(w,w)-z*z));
                        if (std::abs(std::hypot(rho-Rmaj, z)-rmin) > rmin*1e-4 + 1e-6) is_torus=false;
                    }
                    if (is_torus) {
                        q_kind=3; q_C=Ct; q_Zs=Zt; q_R=Rmaj; q_r2=rmin;
                        q_Xs=Vector(a1[0]/Rmaj,a1[1]/Rmaj,a1[2]/Rmaj); q_Ys=cross(q_Zs,q_Xs);
                    }
                }
            }
        }

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
            if (std::getenv("SESSION_VOL_DBG2")) {
                for (int li : face.loop_indices) {
                    if (li < 0 || li >= (int)m_loops.size()) continue;
                    for (int ti : m_loops[li].trim_indices) {
                        if (ti < 0 || ti >= (int)m_trims.size()) continue;
                        int c2 = m_trims[ti].curve_2d_index;
                        if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
                        const NurbsCurve& pcv = m_curves_2d[c2];
                        auto dcv = pcv.domain();
                        for (int k = 0; k <= 400; ++k) {
                            Point q = pcv.point_at(dcv.first + (dcv.second-dcv.first)*k/400.0);
                            Point q3 = srf.point_at(q[0], q[1]);
                            std::fprintf(stderr, "[LQ] %d %d %d %.9f %.9f %.9f\n",
                                (int)(&face - &m_faces[0]), li, ti, q3[0], q3[1], q3[2]);
                        }
                    }
                }
            }
            // (Supporting-plane outward evidence is folded into the PASS 0 shell-orientation
            // vote above -- the propagated fsign already reflects it, weight 3.)
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
                    int n = std::min(std::max(pc.cv_count()*4, 12), 1024);
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
            // the analytic boundary-integral flux; this Gauss is the fallback for other curved faces
            // -- e.g. torus patches trimmed by a section curve, where NU=384 brings the residual
            // staircase under the section-pcurve fit floor (sphere-x-torus common: 1.19e-6 -> 8.6e-7).)
            auto [_du0,_du1] = srf.domain(0); auto [_dv0,_dv1] = srf.domain(1);
            bool rect_trim = inner_polys.empty()
                && std::abs(umin-_du0) < (_du1-_du0)*1e-3 && std::abs(umax-_du1) < (_du1-_du0)*1e-3
                && std::abs(vmin-_dv0) < (_dv1-_dv0)*1e-3 && std::abs(vmax-_dv1) < (_dv1-_dv0)*1e-3;
            if (rect_trim) {
                // NU=24 is exact only when EVERY Gauss point is in-mask. A full-bbox loop that
                // excludes a region (a sphere remainder whose one outer loop spans pole-to-pole
                // around a cap bite) needs the fine masked grid like any curved mask boundary.
                double oa = 0.0;
                for (auto& op2 : outer_polys) {
                    double a2 = 0.0;
                    for (size_t i = 0, j = op2.size()-1; i < op2.size(); j = i++)
                        a2 += op2[j][0]*op2[i][1] - op2[i][0]*op2[j][1];
                    oa += std::abs(0.5*a2);
                }
                if (oa < 0.999*(_du1-_du0)*(_dv1-_dv0)) rect_trim = false;
            }
            curved_rect = rect_trim;
            int NU = rect_trim ? 24 : 384; int NV = NU;
            static const bool s_force_gauss = (std::getenv("SESSION_FORCE_GAUSS") != nullptr);
            if (q_kind != 0 && !rect_trim && !s_force_gauss) NU = 0;  // analytic flux overrides -> skip Gauss
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
            // FLAG-DRIVEN quadric boundary flux (OCCT BRepGProp contract): trims are traversed in
            // stored loop order with their `reversed` flags applied (validated invariant: material
            // LEFT in UV, outer CCW / holes CW). No chaining, no point-in-polygon, no containment:
            // holes subtract via orientation, seam mates cancel pairwise, degenerate edges vanish.
            // Per quadric the UV integral Green-reduces to a closed-form antiderivative carried
            // along the boundary; the trim's exact rational 3D section arc is sampled when the
            // edge carries one (co-orientation resolved at the quarter point).
            auto [su0,su1]=srf.domain(0); auto [sv0,sv1]=srf.domain(1);
            auto dot3=[](const Vector&a,const Vector&b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];};
            const double PI = 3.14159265358979323846;

            int kind = q_kind;                  // 1=sphere, 2=cylinder, 3=torus (hoisted above)
            Point C = q_C; Vector Zs = q_Zs, Xs = q_Xs, Ys = q_Ys;
            double R = q_R, r2 = q_r2;
            if (kind) {
                have_analytic = true;
                // chart handedness (u->angle, v->second coordinate) and natural-normal sign,
                // computed once per face so transformed/mirrored primitives stay correct.
                auto theta_of = [&](const Point& P){ Vector w(P[0]-C[0],P[1]-C[1],P[2]-C[2]);
                    return std::atan2(dot3(w,Ys), dot3(w,Xs)); };
                double uda = theta_of(srf.point_at(su0+0.02*(su1-su0), 0.5*(sv0+sv1)));
                double udb = theta_of(srf.point_at(su0, 0.5*(sv0+sv1)));
                double dth0 = uda-udb; while(dth0> PI)dth0-=2*PI; while(dth0<-PI)dth0+=2*PI;
                double u2th = dth0 >= 0 ? 1.0 : -1.0;
                double CX = C[0]*Xs[0]+C[1]*Xs[1]+C[2]*Xs[2];
                double CY = C[0]*Ys[0]+C[1]*Ys[1]+C[2]*Ys[2];
                double CZ = C[0]*Zs[0]+C[1]*Zs[1]+C[2]*Zs[2];
                double v2h, natout;
                {
                    Point pa = srf.point_at(su0, sv0), pb = srf.point_at(su0, sv1);
                    if (kind == 3) {
                        auto vhat_of = [&](const Point& P){ Vector w(P[0]-C[0],P[1]-C[1],P[2]-C[2]);
                            double z=dot3(w,Zs); double rho=std::sqrt(std::max(0.0,dot3(w,w)-z*z));
                            return std::atan2(z/r2, (rho-R)/r2); };
                        double va = vhat_of(srf.point_at(su0, sv0+0.02*(sv1-sv0)));
                        double vb = vhat_of(pa);
                        double dv0 = va-vb; while(dv0> PI)dv0-=2*PI; while(dv0<-PI)dv0+=2*PI;
                        v2h = dv0 >= 0 ? 1.0 : -1.0;
                    } else {
                        double ha = dot3(Vector(pb[0]-C[0],pb[1]-C[1],pb[2]-C[2]), Zs);
                        double hb = dot3(Vector(pa[0]-C[0],pa[1]-C[1],pa[2]-C[2]), Zs);
                        v2h = (ha - hb) >= 0 ? 1.0 : -1.0;
                    }
                    Point pm2 = srf.point_at(0.5*(su0+su1), 0.5*(sv0+sv1));
                    Vector nm = srf.normal_at(0.5*(su0+su1), 0.5*(sv0+sv1));
                    Vector w(pm2[0]-C[0], pm2[1]-C[1], pm2[2]-C[2]);
                    Vector nref;
                    if (kind == 1) nref = w;                                  // radial
                    else if (kind == 2) { double z=dot3(w,Zs); nref = Vector(w[0]-z*Zs[0],w[1]-z*Zs[1],w[2]-z*Zs[2]); }
                    else { double z=dot3(w,Zs); double rho=std::sqrt(std::max(1e-30,dot3(w,w)-z*z));
                        double px=dot3(w,Xs), py=dot3(w,Ys);
                        Vector er(( px*Xs[0]+py*Ys[0])/rho, (px*Xs[1]+py*Ys[1])/rho, (px*Xs[2]+py*Ys[2])/rho);
                        double cvv=(rho-R)/r2, svv=z/r2;
                        nref = Vector(cvv*er[0]+svv*Zs[0], cvv*er[1]+svv*Zs[1], cvv*er[2]+svv*Zs[2]); }
                    natout = dot3(nm, nref) >= 0 ? 1.0 : -1.0;
                }
                double S = u2th * v2h * natout;   // material-left UV traversal -> natural flux sign

                for (int li : face.loop_indices) {
                    if (li<0||li>=(int)m_loops.size()) continue;
                    // flag-ordered sample stream of the loop boundary
                    std::vector<Point> p3s;
                    std::vector<char> vlock;    // per-sample: 0 free, 1/2 v-locked, 3 pole run
                    std::vector<double> pu;     // pcurve u per sample (theta source on pole runs)
                    auto d2p=[](const Point&a,const Point&b){double dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2];return dx*dx+dy*dy+dz*dz;};
                    for (int ti : m_loops[li].trim_indices) {
                        if (ti<0||ti>=(int)m_trims.size()) continue;
                        int c2=m_trims[ti].curve_2d_index; if(c2<0||c2>=(int)m_curves_2d.size()) continue;
                        const NurbsCurve& pc=m_curves_2d[c2]; if(!pc.is_valid()) continue;
                        auto dc=pc.domain();
                        bool fwd = !m_trims[ti].reversed;
                        Point uvS2=pc.point_at(fwd?dc.first:dc.second), uvE2=pc.point_at(fwd?dc.second:dc.first);
                        const NurbsCurve* c3=nullptr; double c3a=0,c3b=0; int e=m_trims[ti].edge_index;
                        if(e>=0&&e<(int)m_topology_edges.size()){ int c3i=m_topology_edges[e].curve_3d_index;
                            if(c3i>=0&&c3i<(int)m_curves_3d.size()){ c3=&m_curves_3d[c3i]; auto d3=c3->domain(); c3a=d3.first; c3b=d3.second; } }
                        // A v=const seg's OUTER coordinate is constant (phi/h/vhat all map from v):
                        // LOCK it to the seg-start value so off-surface sampling jitter cannot
                        // couple to the big theta-linear antiderivative, while theta samples still
                        // carry the unwrap/winding through the seg.
                        bool lock_v = false;
                        if (std::abs(uvS2[1]-uvE2[1]) < 1e-9*(sv1-sv0)) {
                            double tmc=0.5*(dc.first+dc.second);
                            if (std::abs(pc.point_at(tmc)[1]-uvS2[1]) < 1e-9*(sv1-sv0)) lock_v = true;
                        }
                        // POLE RUN (sphere): a 3D-degenerate seg is the chart's pole line; theta
                        // is undefined there geometrically, but the chart boundary's theta ALONG the
                        // pole line is the pcurve's u -- carry it so the two seam runs it connects
                        // keep their 2*pi offset (the winding term of every pole-adjacent face).
                        if (kind == 1) {
                            Point pa3=srf.point_at(uvS2[0],uvS2[1]), pb3=srf.point_at(uvE2[0],uvE2[1]);
                            double tmc0=0.5*(dc.first+dc.second); Point uvm0=pc.point_at(tmc0);
                            Point pm30=srf.point_at(uvm0[0],uvm0[1]);
                            double ext=std::max(pa3.distance(pb3), pa3.distance(pm30));
                            if (ext < R*1e-7) {
                                for(int s=0;s<200;++s){ double f=(double)s/200.0;
                                    double tt=fwd?dc.first+(dc.second-dc.first)*f:dc.second-(dc.second-dc.first)*f;
                                    Point uv=pc.point_at(tt);
                                    p3s.push_back(pa3); vlock.push_back(3); pu.push_back(uv[0]); }
                                continue;
                            }
                        }
                        bool use_c3=false, c3_rev=false;
                        // Only an exact RATIONAL section arc is worth sampling directly: polyline
                        // 3D curves lie off the surface (chord sag), and on a torus that off-surface
                        // jitter couples to the large theta-linear antiderivative term as a random
                        // walk (~0.5 flux on a band). Surface-evaluated pcurve samples are always
                        // exactly on-surface. Gate on BOTH endpoints plus a midpoint sanity check:
                        // neighbouring quarters share an endpoint and full halves share both, so a
                        // cross-assigned edge (its curve is the OTHER arc) passes a one-endpoint
                        // gate but sits an arc-diameter away at the middle; the pcurve fallback is
                        // geometrically correct there.
                        if(c3 && c3->is_rational()){
                            Point q0=srf.point_at(uvS2[0],uvS2[1]);
                            Point q1=srf.point_at(uvE2[0],uvE2[1]);
                            Point ca=c3->point_at(c3a), cb=c3->point_at(c3b);
                            bool fw_ok = d2p(q0,ca)<1e-6 && d2p(q1,cb)<1e-6;
                            bool bw_ok = d2p(q0,cb)<1e-6 && d2p(q1,ca)<1e-6;
                            if (fw_ok || bw_ok) {
                                double tmq=0.5*(dc.first+dc.second); Point uvq=pc.point_at(tmq);
                                Point pmq=srf.point_at(uvq[0],uvq[1]);
                                Point cmq=c3->point_at(0.5*(c3a+c3b));
                                double chord = std::max(ca.distance(cb), ca.distance(cmq)) + 1e-12;
                                if (pmq.distance(cmq) < 0.35*chord + 1e-6) {
                                    use_c3=true;
                                    if (fw_ok && bw_ok) {
                                        // closed loop: endpoints carry no direction, quarter decides
                                        double tq=fwd?dc.first+(dc.second-dc.first)*0.25:dc.second-(dc.second-dc.first)*0.25;
                                        Point uvq4=pc.point_at(tq);
                                        Point uvQ=srf.point_at(uvq4[0],uvq4[1]);
                                        double qf=d2p(uvQ, c3->point_at(fwd?c3a+(c3b-c3a)*0.25:c3b-(c3b-c3a)*0.25));
                                        double qr=d2p(uvQ, c3->point_at(fwd?c3b-(c3b-c3a)*0.25:c3a+(c3b-c3a)*0.25));
                                        c3_rev = qr < qf;
                                    } else {
                                        // open arc: cf=(fwd!=c3_rev) must walk c3a->c3b iff fw_ok
                                        c3_rev = fw_ok ? !fwd : fwd;
                                    }
                                }
                            } }
                        // corner error of the boundary trapezoid is O((span/n)^2 * |dG'|): 200
                        // floors sphere fuse remainders at ~9e-6 rel; 2000 puts corners ~1e-8.
                        if (!use_c3 && pc.degree() == 1 && !pc.is_rational() && pc.cv_count() >= 2) {
                            // deg-1 pcurve: its vertices ARE the geometry -- walk them directly
                            // (uniform point_at resampling at a fixed count ALIASES the polyline
                            // back to coarse secants); subdivide segments linearly in UV to keep
                            // the trapezoid corner-error floor (>= ~2000 samples per trim).
                            int k = pc.cv_count();
                            int m = std::max(1, (2000 + k - 2) / (k - 1));
                            bool first_s = true;
                            for (int s = 0; s < k - 1; ++s) {
                                int i0 = fwd ? s : k - 1 - s, i1 = fwd ? s + 1 : k - 2 - s;
                                Point a = pc.get_cv(i0), b = pc.get_cv(i1);
                                for (int q = 0; q < m; ++q) {
                                    double f = (double)q / m;
                                    p3s.push_back(srf.point_at(a[0]+(b[0]-a[0])*f, a[1]+(b[1]-a[1])*f));
                                    vlock.push_back(lock_v ? (first_s ? 2 : 1) : 0); pu.push_back(0.0);
                                    first_s = false;
                                }
                            }
                            continue;
                        }
                        // resampling a polyline pcurve at FEWER points than its vertex count
                        // aliases it back to coarse secants -- track its density
                        int n = (use_c3 && c3->is_rational()) ? 4000 : std::max(2000, pc.cv_count()*4);
                        for(int s=0;s<n;++s){double f=(double)s/n;
                            if(use_c3){ bool cf=(fwd!=c3_rev);
                                double t3=cf?c3a+(c3b-c3a)*f:c3b-(c3b-c3a)*f;
                                p3s.push_back(c3->point_at(t3)); }
                            else { double tt=fwd?dc.first+(dc.second-dc.first)*f:dc.second-(dc.second-dc.first)*f;
                                Point uv=pc.point_at(tt); p3s.push_back(srf.point_at(uv[0],uv[1])); }
                            vlock.push_back(lock_v ? (s==0 ? 2 : 1) : 0); pu.push_back(0.0); }
                    }
                    if(p3s.size()<3) continue;
                    // Never START the traversal on a pole run: the pole has no longitude, so a
                    // loop anchored there takes an arbitrary theta and lands the pole-exit half a
                    // turn off the seam it must meet; rotating to a normal seg gives every pole a
                    // proper incoming theta to carry from.
                    {
                        size_t s0 = 0;
                        auto at_pole = [&](size_t i){
                            if (kind != 1) return false;
                            double zz=(p3s[i][0]-C[0])*Zs[0]+(p3s[i][1]-C[1])*Zs[1]+(p3s[i][2]-C[2])*Zs[2];
                            return std::abs(zz) >= R*(1.0-1e-8);
                        };
                        while (s0 < vlock.size() && (vlock[s0] == 3 || at_pole(s0))) ++s0;
                        if (s0 > 0 && s0 < vlock.size()) {
                            std::rotate(p3s.begin(), p3s.begin()+s0, p3s.end());
                            std::rotate(vlock.begin(), vlock.begin()+s0, vlock.end());
                            std::rotate(pu.begin(), pu.begin()+s0, pu.end());
                        }
                    }
                    p3s.push_back(p3s.front()); vlock.push_back(vlock.front()); pu.push_back(pu.front());
                    // accumulate the per-kind Green boundary integral over the ordered samples
                    double I=0.0, wind=0.0;
                    double prevTh=0.0, prevH=0.0, prevG=0.0, lockval=0.0; bool first=true;
                    double prev_pu = 0.0; bool prev_pole = false;
                    for(size_t i=0;i<p3s.size();++i){
                        const Point& P=p3s[i];
                        Vector w(P[0]-C[0], P[1]-C[1], P[2]-C[2]);
                        double z = dot3(w,Zs);
                        double th;
                        if (vlock[i] == 3) {
                            th = first ? 0.0
                               : (prev_pole ? prevTh + u2th*(pu[i]-prev_pu)*2.0*PI/(su1-su0)
                                            : prevTh);
                            if(!first) wind += th - prevTh;
                            prev_pu = pu[i]; prev_pole = true;
                        } else if (kind == 1 && std::abs(z) >= R*(1.0-1e-8)) {
                            // sample sits ON the pole (a seam run's pole-side endpoint): longitude
                            // is undefined and G vanishes there (cos phi factor); hold theta so the
                            // unwrap continuity survives.
                            th = first ? 0.0 : prevTh;
                            prev_pole = false;
                        } else {
                            th = std::atan2(dot3(w,Ys), dot3(w,Xs));
                            if(!first){ double dth=th-prevTh;
                                while(dth> PI)dth-=2*PI; while(dth<-PI)dth+=2*PI;
                                th=prevTh+dth; wind+=dth; }
                            prev_pole = false;
                        }
                        if (kind == 1) {
                            // latitude-direction Green: G = int f dtheta with
                            // f = (C.n + R) R^2 cos(phi); pole runs have dphi = 0 by construction,
                            // seam runs carry the theta-winding term -- no pole special-casing.
                            double phi = std::asin(std::max(-1.0, std::min(1.0, z / R)));
                            if(vlock[i] == 2) lockval = phi;
                            else if(vlock[i] == 1) phi = lockval;
                            double cph = std::cos(phi);
                            double G = R*R*cph*( CX*cph*std::sin(th) - CY*cph*std::cos(th)
                                               + (CZ*std::sin(phi) + R)*th );
                            if(!first) I += 0.5*(G+prevG)*(phi-prevH);
                            prevG=G; prevH=phi;
                        } else if (kind == 2) {
                            // v=const seg: the seg's first sample sets the level (the corner step
                            // from the previous seg is real); the rest lock to it so jitter -> 0.
                            if(vlock[i] == 2) lockval = z;
                            else if(vlock[i] == 1) z = lockval;
                            double G = R*R*th + R*(CX*std::sin(th) - CY*std::cos(th));
                            if(!first) I += 0.5*(G+prevG)*(z-prevH);
                            prevG=G; prevH=z;
                        } else {
                            double rho=std::sqrt(std::max(0.0, dot3(w,w)-z*z));
                            double cvv=(rho-R)/r2, svv=z/r2;
                            double vh=std::atan2(svv,cvv);
                            if(!first){ double dvh=vh-prevH;
                                while(dvh> PI)dvh-=2*PI; while(dvh<-PI)dvh+=2*PI; vh=prevH+dvh; }
                            if(vlock[i] == 2) lockval = vh;
                            else if(vlock[i] == 1) vh = lockval;
                            double ring = r2*(R + r2*cvv);
                            double G = ring*( cvv*(CX*std::sin(th) - CY*std::cos(th))
                                            + (CZ*svv + (R + r2*cvv)*cvv + r2*svv*svv)*th );
                            if(!first) I += 0.5*(G+prevG)*(vh-prevH);
                            prevG=G; prevH=vh;
                        }
                        prevTh=th; first=false;
                    }
                    flux_analytic += S * I;
                }
            }
        }
        // Chart integrals are exact for recognized quadrics with REAL boundary loops; a
        // full-domain rect face's seam boundaries cancel pairwise (I = 0), so the rect Gauss
        // stays authoritative there (rect_trim is coverage-gated: masked faces never qualify).
        double gauss_val = flux_nat;
        if (have_analytic && !curved_rect && !pole_face) flux_nat = flux_analytic;
        if (std::getenv("SESSION_VOL_DBG"))
            std::fprintf(stderr,"[VOLDBG] f=%ld planar=%d haveA=%d rect=%d pole=%d sign=%.0f gauss=%.6f an=%.6f\n",
                (long)(&face-&m_faces[0]), is_planar(srf)?1:0, have_analytic?1:0, curved_rect?1:0, pole_face?1:0, sign, gauss_val, flux_analytic);
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
        // one operand's section copy is now fit-accurate (~1e-5); the other's lift must
        // stay within the sew tolerance of it, not merely within 2e-3 of itself
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

        // Interior vertices that lie on C (a T-junction split point). The end-guard and
        // dedup use a PAVE tolerance (curve-size-relative, OCCT PutPavesOnCurve): the same
        // geometric event reached through both operands' lifts differs by the lift
        // tolerance (~1e-4), and splitting that close to an end spawns a micro-stub edge
        // that can never mate (spiric tips).
        double pave_tol = std::max(tol, std::sqrt((ebb[3]-ebb[0])*(ebb[3]-ebb[0])
            + (ebb[4]-ebb[1])*(ebb[4]-ebb[1]) + (ebb[5]-ebb[2])*(ebb[5]-ebb[2])) * 2e-4);
        struct Split { double tc; Point V; };
        std::vector<Split> splits;
        for (const Point& V : vpos) {
            if (V[0] > 1e299) continue;
            if (V[0] < ebb[0]-tol || V[0] > ebb[3]+tol || V[1] < ebb[1]-tol || V[1] > ebb[4]+tol
                || V[2] < ebb[2]-tol || V[2] > ebb[5]+tol) continue;
            if (V.distance(pA) < pave_tol || V.distance(pB) < pave_tol) continue;
            double tc = C.closest_parameter(V);
            if (C.point_at(tc).distance(V) > tol) continue;
            auto dom = C.domain();
            double frac = (tc - dom.first) / (dom.second - dom.first);
            if (frac <= 1e-6 || frac >= 1.0 - 1e-6) continue;
            // Dedup near-equal split points.
            bool dup = false;
            for (auto& s : splits) if (s.V.distance(V) < pave_tol) { dup = true; break; }
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
                // branch-free split param (see co_refine splice): project the 3D split point
                // onto the pcurve's surface image, never through Closest's principal branch.
                auto param_near_3d = [&](const NurbsCurve& P2, const NurbsSurface& S2, const Point& V) -> double {
                    auto pd = P2.domain(); int M = std::min(std::max(64, P2.cv_count()*4), 4096);
                    double bt = pd.first, bd = 1e300; Point prev3(0,0,0); double prevt = pd.first;
                    for (int i2 = 0; i2 <= M; ++i2) {
                        double t = pd.first + (pd.second - pd.first)*i2/M;
                        Point uv = P2.point_at(t); Point q = S2.point_at(uv[0], uv[1]);
                        if (i2 > 0) {
                            double ex=q[0]-prev3[0], ey=q[1]-prev3[1], ez=q[2]-prev3[2];
                            double L2=ex*ex+ey*ey+ez*ez;
                            double lam=(L2>1e-30)?((V[0]-prev3[0])*ex+(V[1]-prev3[1])*ey+(V[2]-prev3[2])*ez)/L2:0.0;
                            lam=std::min(std::max(lam,0.0),1.0);
                            double cx=prev3[0]+lam*ex, cy=prev3[1]+lam*ey, cz=prev3[2]+lam*ez;
                            double d=(V[0]-cx)*(V[0]-cx)+(V[1]-cy)*(V[1]-cy)+(V[2]-cz)*(V[2]-cz);
                            if (d<bd){ bd=d; bt=prevt+lam*(t-prevt); }
                        }
                        prev3=q; prevt=t;
                    }
                    auto d3at = [&](double t){ Point uv=P2.point_at(t); Point q=S2.point_at(uv[0],uv[1]);
                        double dx=q[0]-V[0], dy=q[1]-V[1], dz=q[2]-V[2]; return dx*dx+dy*dy+dz*dz; };
                    double lo = std::max(pd.first,  bt - (pd.second-pd.first)/M);
                    double hi = std::min(pd.second, bt + (pd.second-pd.first)/M);
                    for (int it = 0; it < 80 && hi-lo > 1e-14*(pd.second-pd.first); ++it) {
                        double m1 = lo + (hi-lo)/3, m2 = hi - (hi-lo)/3;
                        if (d3at(m1) < d3at(m2)) hi = m2; else lo = m1;
                    }
                    return 0.5*(lo+hi);
                };
                std::vector<double> tps;
                for (auto& s : splits) tps.push_back(param_near_3d(P, S, s.V));
                p2pieces = split_multi(P, tps);
            }
            bool ok = ((int)p2pieces.size() == (int)edge_ids.size());
            // A MATE-oriented trim's pcurve runs opposite the shared 3D curve (the edge was
            // lifted from the OTHER face's pcurve), so pieces in pcurve-param order do NOT line
            // up index-for-index with the 3D pieces -- index pairing hands each trim a WRONG edge
            // (the cross-assigned-edge corruption). Match each piece to ITS edge geometrically,
            // endpoint-compatible with a midpoint tie-break (same fix as co_refine's splice).
            std::vector<int> edge_for(edge_ids.begin(), edge_ids.end());
            if (ok && si >= 0 && si < (int)m_surfaces.size()) {
                const NurbsSurface& S2 = m_surfaces[si];
                double mtol = tol * 1e3;
                std::vector<int> ef(p2pieces.size(), -1);
                std::vector<bool> taken(c3pieces.size(), false);
                bool allm = true;
                for (size_t j = 0; j < p2pieces.size() && allm; ++j) {
                    if (!p2pieces[j].is_valid()) { allm = false; break; }
                    auto pd = p2pieces[j].domain();
                    Point a2u = p2pieces[j].point_at(pd.first), b2u = p2pieces[j].point_at(pd.second);
                    Point m2u = p2pieces[j].point_at(0.5*(pd.first+pd.second));
                    Point a3 = S2.point_at(a2u[0],a2u[1]), b3 = S2.point_at(b2u[0],b2u[1]);
                    Point m3 = S2.point_at(m2u[0],m2u[1]);
                    int hit = -1; double bestm = 1e300;
                    for (size_t k = 0; k < c3pieces.size(); ++k) {
                        if (taken[k]) continue;
                        Point ea = c3pieces[k].point_at_start(), eb = c3pieces[k].point_at_end();
                        bool epok = (a3.distance(ea) < mtol && b3.distance(eb) < mtol)
                                 || (a3.distance(eb) < mtol && b3.distance(ea) < mtol);
                        if (!epok) continue;
                        auto ad2 = c3pieces[k].domain();
                        Point em = c3pieces[k].point_at(0.5*(ad2.first+ad2.second));
                        double dm = m3.distance(em);
                        if (dm < bestm) { bestm = dm; hit = (int)k; }
                    }
                    if (hit < 0) { allm = false; break; }
                    taken[hit] = true; ef[j] = edge_ids[hit];
                }
                if (allm) edge_for.assign(ef.begin(), ef.end());
            }
            // Build the per-piece trims (forward order; boolean trims are not reversed).
            std::vector<int> newtrims;
            for (size_t j = 0; j < edge_ids.size(); ++j) {
                int c2idx, tidx;
                NurbsCurve pc = ok ? p2pieces[j] : (j == 0 ? m_curves_2d[std::max(c2,0)] : NurbsCurve());
                if (j == 0) { c2idx = (c2 >= 0 ? c2 : add_curve_2d(pc)); if (c2 >= 0) m_curves_2d[c2] = pc; tidx = ti; }
                else { c2idx = add_curve_2d(pc); tidx = (int)m_trims.size(); m_trims.push_back(BRepTrim()); }
                m_trims[tidx].curve_2d_index = c2idx;
                m_trims[tidx].edge_index = edge_for[j];
                m_trims[tidx].loop_index = li;
                m_trims[tidx].reversed = T.reversed;
                m_trims[tidx].type = T.type;
                newtrims.push_back(tidx);
                m_topology_edges[edge_for[j]].trim_indices.push_back(tidx);
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

namespace {
// Exact rational-quadratic arc on the circle (Cc, e1, e2, r): starts at angle a0, sweeps `swept`
// radians (signed), split into <=90-deg Bezier segments with the standard w = cos(phi/2) mid-CV
// (same construction as intersection.cpp exact_circle). Endpoints reproduce on-circle points
// exactly, so arcs built from shared snapped vertices are bit-identical on both operands.
NurbsCurve exact_arc_3d(const Point& Cc, const Vector& e1, const Vector& e2, double r,
                        double a0, double swept) {
    const double PI = 3.14159265358979323846;
    int nseg = (int)std::ceil(std::abs(swept) / (PI / 2.0) - 1e-12);
    if (nseg < 1) nseg = 1;
    double phi = swept / nseg;
    double w = std::cos(0.5 * phi);
    int ncv = 2 * nseg + 1;
    NurbsCurve crv(3, true, 3, ncv);
    for (int i = 0; i < ncv + 1; ++i) crv.set_nurbsknot(i, (double)(i / 2));
    auto onc = [&](double a, double rr) {
        return Point(Cc[0] + rr * (std::cos(a) * e1[0] + std::sin(a) * e2[0]),
                     Cc[1] + rr * (std::cos(a) * e1[1] + std::sin(a) * e2[1]),
                     Cc[2] + rr * (std::cos(a) * e1[2] + std::sin(a) * e2[2]));
    };
    Point P0 = onc(a0, r);
    crv.set_cv_4d(0, P0[0], P0[1], P0[2], 1.0);
    for (int s = 0; s < nseg; ++s) {
        double aa = a0 + phi * s, ab = aa + phi, am = 0.5 * (aa + ab);
        Point M = onc(am, r / std::cos(0.5 * phi));
        Point P1 = onc(ab, r);
        crv.set_cv_4d(2 * s + 1, M[0] * w, M[1] * w, M[2] * w, w);
        crv.set_cv_4d(2 * s + 2, P1[0], P1[1], P1[2], 1.0);
    }
    crv.set_domain(0.0, 1.0);
    return crv;
}

// Circle through 3D points: circumcenter from three spread samples, then EVERY point must lie on
// that circle (planarity AND radius) within a tight relative tolerance. The lift polylines'
// vertices are exact on-circle surface evaluations, so genuine section circles pass with ~1e-15
// residual while lines/ellipses/cone conics/freeform sections fail immediately.
// Exact sphere recognition from the surface itself (poles -> center/radius, verified on a
// sample grid). Our sphere NURBS are exact rational surfaces, so this is machine precision --
// unlike the pullback-interpolated lift vertices, which sit ~1e-4 off the section circle.
bool sphere_of_surface(const NurbsSurface& s, Point& C, double& R) {
    auto du = s.domain(0); auto dv = s.domain(1);
    double um = 0.5*(du.first + du.second);
    Point ps = s.point_at(um, dv.first), pn = s.point_at(um, dv.second);
    C = Point(0.5*(ps[0]+pn[0]), 0.5*(ps[1]+pn[1]), 0.5*(ps[2]+pn[2]));
    R = 0.5*ps.distance(pn);
    if (R < 1e-9) return false;
    for (int i = 0; i <= 3; ++i) for (int j = 1; j < 3; ++j) {
        Point p = s.point_at(du.first+(du.second-du.first)*i/3.0, dv.first+(dv.second-dv.first)*j/3.0);
        if (std::abs(p.distance(C) - R) > R*1e-7 + 1e-9) return false;
    }
    return true;
}

// Exact cylinder recognition: the v-boundary circles' centers give the axis (the rational-
// quadratic circle's antipode sits exactly at mid-u), verified on a sample grid.
bool cylinder_of_surface(const NurbsSurface& s, Point& A, Vector& W, double& R) {
    auto du = s.domain(0); auto dv = s.domain(1);
    double um2 = 0.5*(du.first + du.second);
    Point a0 = s.point_at(du.first, dv.first), a1 = s.point_at(um2, dv.first);
    Point b0 = s.point_at(du.first, dv.second), b1 = s.point_at(um2, dv.second);
    A = Point(0.5*(a0[0]+a1[0]), 0.5*(a0[1]+a1[1]), 0.5*(a0[2]+a1[2]));
    Point C1(0.5*(b0[0]+b1[0]), 0.5*(b0[1]+b1[1]), 0.5*(b0[2]+b1[2]));
    double wl = A.distance(C1);
    if (wl < 1e-9) return false;
    W = Vector((C1[0]-A[0])/wl, (C1[1]-A[1])/wl, (C1[2]-A[2])/wl);
    R = 0.5*a0.distance(a1);
    if (R < 1e-9) return false;
    for (int i = 0; i <= 3; ++i) for (int j = 0; j <= 2; ++j) {
        Point p = s.point_at(du.first+(du.second-du.first)*i/3.0, dv.first+(dv.second-dv.first)*j/2.0);
        double wx=p[0]-A[0], wy=p[1]-A[1], wz=p[2]-A[2];
        double t = wx*W[0]+wy*W[1]+wz*W[2];
        double dx=wx-t*W[0], dy=wy-t*W[1], dz=wz-t*W[2];
        if (std::abs(std::sqrt(dx*dx+dy*dy+dz*dz) - R) > R*1e-7 + 1e-9) return false;
    }
    return true;
}

bool circle_through_points(const std::vector<Point>& pts, Point& Cc, Vector& nrm,
                           Vector& e1, Vector& e2, double& r) {
    int m = (int)pts.size();
    if (m < 4) return false;
    const Point& p0 = pts[0];
    const Point& p1 = pts[m / 3];
    const Point& p2 = pts[(2 * m) / 3];
    double ax = p1[0]-p0[0], ay = p1[1]-p0[1], az = p1[2]-p0[2];
    double bx = p2[0]-p0[0], by = p2[1]-p0[1], bz = p2[2]-p0[2];
    double nx = ay*bz - az*by, ny = az*bx - ax*bz, nz = ax*by - ay*bx;
    double n2 = nx*nx + ny*ny + nz*nz;
    if (n2 < 1e-20) return false;
    // circumcenter = p0 + (|b|^2 ((a x b) x a) + |a|^2 (b x (a x b))) / (2 |a x b|^2)
    double a2 = ax*ax + ay*ay + az*az, b2 = bx*bx + by*by + bz*bz;
    double c1x = ny*az - nz*ay, c1y = nz*ax - nx*az, c1z = nx*ay - ny*ax;   // (a x b) x a
    double c2x = by*nz - bz*ny, c2y = bz*nx - bx*nz, c2z = bx*ny - by*nx;   // b x (a x b)
    Cc = Point(p0[0] + (b2*c1x + a2*c2x) / (2.0*n2),
               p0[1] + (b2*c1y + a2*c2y) / (2.0*n2),
               p0[2] + (b2*c1z + a2*c2z) / (2.0*n2));
    r = Cc.distance(p0);
    if (r < 1e-12) return false;
    double nl = std::sqrt(n2);
    nrm = Vector(nx/nl, ny/nl, nz/nl);
    e1 = Vector((p0[0]-Cc[0])/r, (p0[1]-Cc[1])/r, (p0[2]-Cc[2])/r);
    e2 = Vector(nrm[1]*e1[2]-nrm[2]*e1[1], nrm[2]*e1[0]-nrm[0]*e1[2], nrm[0]*e1[1]-nrm[1]*e1[0]);
    double rtol = std::max(1e-7 * r, 1e-9);
    for (const auto& p : pts) {
        double wx = p[0]-Cc[0], wy = p[1]-Cc[1], wz = p[2]-Cc[2];
        double h = wx*nrm[0] + wy*nrm[1] + wz*nrm[2];
        if (std::abs(h) > rtol) return false;
        double rad = std::sqrt(wx*wx + wy*wy + wz*wz);
        if (std::abs(rad - r) > rtol) return false;
    }
    return true;
}
} // namespace

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
    // NS=24 under-resolves an elongated closed section loop: a 25-point polyline of the box x tor
    // spiric (3.9 x 1.1 lens) SAGS ~0.034 at the high-curvature tips while the true arc-to-arc
    // deviation is 0.0036 -- subset_of then fails on SAMPLING error alone (tol=diag*5e-3=0.029),
    // the closed disc edge is never split at its arcs' feet, and the result is not solid. Denser
    // sampling only reveals true proximity (measurement-side): it can only make coincidence tests
    // MORE accurate, never merge distinct edges.
    const int NS = 96;
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
        // EXACT SECTION-CIRCLE REBUILD (P0): the lift stores every 3D edge as a chord polyline
        // (devtol ~2e-3 of the face size), so a split section CIRCLE - and every pcurve rebuilt
        // from its pieces - INSCRIBES the true circle: O(1/N^2) area/volume deficit (box x sphere
        // ~1.5e-3). The polyline vertices are exact on-circle points, so recover the true circle,
        // snap the split points onto it, and below rebuild each piece as the EXACT rational arc
        // between its snapped vertices. Non-circles fail the fit and keep the polyline path.
        // Recognized section conic: center fitC, unit axes fitE1/fitE2 with semi-axes fitA/fitB
        // (circle: fitA == fitB). All arc math below runs in the SCALED frame (sx,sy) where the
        // conic is the unit circle -- a rational-quadratic arc is exact under the linear map, so
        // exact_arc_3d with basis (fitE1*fitA, fitE2*fitB, r=1) emits exact ELLIPSE arcs too.
        Point fitC; Vector fitN, fitE1, fitE2; double fitR = 0, fitA = 0, fitB = 0;
        bool fit_ok = false;
        if (C.degree() == 1 && !C.is_rational()) {
            std::vector<Point> cpts;
            for (int i = 0; i < C.cv_count(); ++i) cpts.push_back(C.get_cv(i));
            fit_ok = circle_through_points(cpts, fitC, fitN, fitE1, fitE2, fitR);
        }
        if (!fit_ok && C.degree() == 1 && !C.is_rational()) {
            // The vertex fit fails when BOTH operands' lifts are pullback-interpolated (~1e-4
            // noise, e.g. sphere x sphere -- no exact planar side to trust). The operand SURFACES
            // are exact: derive the section circle from the two adjacent spheres' radical plane,
            // then verify the vertices LOOSELY (rejects edges that are not this circle at all).
            auto srf_of_edge = [&](int e) -> int {
                if (e < 0 || e >= (int)m_topology_edges.size()) return -1;
                for (int t : m_topology_edges[e].trim_indices) {
                    if (t < 0 || t >= (int)m_trims.size()) continue;
                    int l = m_trims[t].loop_index; if (l < 0 || l >= (int)m_loops.size()) continue;
                    int f = m_loops[l].face_index; if (f < 0 || f >= (int)m_faces.size()) continue;
                    return m_faces[f].surface_index;
                }
                return -1;
            };
            int s1 = srf_of_edge(ei), s2 = -1;
            for (int ej = 0; ej < ne0 && s2 < 0; ++ej) {
                if (ej == ei || !cand(ej) || bbox_far(ei,ej) || !subset_of(ej,ei)) continue;
                int sj = srf_of_edge(ej);
                if (sj >= 0 && sj != s1) s2 = sj;
            }
            if (s1 >= 0 && s2 >= 0 && s1 < (int)m_surfaces.size() && s2 < (int)m_surfaces.size()) {
                Point C1, C2; double R1 = 0, R2 = 0;
                if (sphere_of_surface(m_surfaces[s1], C1, R1) && sphere_of_surface(m_surfaces[s2], C2, R2)) {
                    double dx=C2[0]-C1[0], dy=C2[1]-C1[1], dz=C2[2]-C1[2];
                    double d = std::sqrt(dx*dx+dy*dy+dz*dz);
                    if (d > 1e-9 && d < R1+R2) {
                        double a = (d*d + R1*R1 - R2*R2) / (2.0*d);
                        double rr2 = R1*R1 - a*a;
                        if (rr2 > 1e-18) {
                            fitN = Vector(dx/d, dy/d, dz/d);
                            fitC = Point(C1[0]+a*fitN[0], C1[1]+a*fitN[1], C1[2]+a*fitN[2]);
                            fitR = std::sqrt(rr2);
                            Point P0 = C.point_at_start();
                            double wx=P0[0]-fitC[0], wy=P0[1]-fitC[1], wz=P0[2]-fitC[2];
                            double h = wx*fitN[0]+wy*fitN[1]+wz*fitN[2];
                            double px=wx-h*fitN[0], py=wy-h*fitN[1], pz=wz-h*fitN[2];
                            double pl = std::sqrt(px*px+py*py+pz*pz);
                            if (pl > 1e-12) {
                                fitE1 = Vector(px/pl, py/pl, pz/pl);
                                fitE2 = Vector(fitN[1]*fitE1[2]-fitN[2]*fitE1[1],
                                               fitN[2]*fitE1[0]-fitN[0]*fitE1[2],
                                               fitN[0]*fitE1[1]-fitN[1]*fitE1[0]);
                                double vtol = std::max(5e-3*fitR, 1e-6);
                                bool okv = true;
                                for (int i = 0; i < C.cv_count() && okv; ++i) {
                                    Point p = C.get_cv(i);
                                    double ax=p[0]-fitC[0], ay=p[1]-fitC[1], az=p[2]-fitC[2];
                                    double hh = ax*fitN[0]+ay*fitN[1]+az*fitN[2];
                                    double rd2 = ax*ax+ay*ay+az*az - hh*hh;
                                    double rad = rd2 > 0 ? std::sqrt(rd2) : 0.0;
                                    if (std::abs(hh) > vtol || std::abs(rad-fitR) > vtol) okv = false;
                                }
                                fit_ok = okv;
                            }
                        }
                    }
                }
                if (!fit_ok && std::getenv("SESSION_BOOL_ELLIPSE")) {
                    // CYLINDER x CYLINDER, equal radii, intersecting axes: the section is TWO
                    // exact ellipses in the (w1 +- w2) planes (Steinmetz); pick the one whose
                    // ellipse the edge's vertices actually lie on. GATED OFF pending P8: the
                    // cylinder faces' masked-Gauss flux dominates the cyl x cyl error anyway,
                    // and enabling this flipped one fuse edge to non-solid (mate mismatch TBD).
                    Point A1, A2; Vector W1, W2; double Rc1 = 0, Rc2 = 0;
                    if (cylinder_of_surface(m_surfaces[s1], A1, W1, Rc1)
                     && cylinder_of_surface(m_surfaces[s2], A2, W2, Rc2)
                     && std::abs(Rc1-Rc2) < 1e-6*std::max(Rc1,Rc2)) {
                        double b = W1[0]*W2[0]+W1[1]*W2[1]+W1[2]*W2[2];
                        double den = 1.0 - b*b;
                        if (den > 1e-12) {
                            double d0x=A2[0]-A1[0], d0y=A2[1]-A1[1], d0z=A2[2]-A1[2];
                            double dw1 = d0x*W1[0]+d0y*W1[1]+d0z*W1[2];
                            double dw2 = d0x*W2[0]+d0y*W2[1]+d0z*W2[2];
                            double t1 = (dw1 - b*dw2)/den, t2 = (b*dw1 - dw2)/den;
                            Point q1(A1[0]+t1*W1[0], A1[1]+t1*W1[1], A1[2]+t1*W1[2]);
                            Point q2(A2[0]+t2*W2[0], A2[1]+t2*W2[1], A2[2]+t2*W2[2]);
                            if (q1.distance(q2) < 1e-6) {
                                double Rr = 0.5*(Rc1+Rc2);
                                double cx=W1[1]*W2[2]-W1[2]*W2[1], cy=W1[2]*W2[0]-W1[0]*W2[2], cz=W1[0]*W2[1]-W1[1]*W2[0];
                                double cl = std::sqrt(cx*cx+cy*cy+cz*cz);
                                double ang = std::atan2(cl, b);
                                double sh = std::sin(0.5*ang), ch = std::cos(0.5*ang);
                                if (cl > 1e-9 && sh > 1e-9 && ch > 1e-9) {
                                    Vector mnr(cx/cl, cy/cl, cz/cl);
                                    Point Pc(0.5*(q1[0]+q2[0]), 0.5*(q1[1]+q2[1]), 0.5*(q1[2]+q2[2]));
                                    for (int cand = 0; cand < 2 && !fit_ok; ++cand) {
                                        double sx = cand==0 ? 1.0 : -1.0;
                                        double mx=W1[0]+sx*W2[0], my=W1[1]+sx*W2[1], mz=W1[2]+sx*W2[2];
                                        double ml = std::sqrt(mx*mx+my*my+mz*mz);
                                        if (ml < 1e-9) continue;
                                        Vector maj(mx/ml, my/ml, mz/ml);
                                        double semiA = cand==0 ? Rr/sh : Rr/ch;
                                        Vector n2(maj[1]*mnr[2]-maj[2]*mnr[1], maj[2]*mnr[0]-maj[0]*mnr[2], maj[0]*mnr[1]-maj[1]*mnr[0]);
                                        double vtol = std::max(5e-3*Rr, 1e-6);
                                        bool okv = true;
                                        for (int i = 0; i < C.cv_count() && okv; ++i) {
                                            Point p = C.get_cv(i);
                                            double ax=p[0]-Pc[0], ay=p[1]-Pc[1], az=p[2]-Pc[2];
                                            double hh = ax*n2[0]+ay*n2[1]+az*n2[2];
                                            double su = (ax*maj[0]+ay*maj[1]+az*maj[2])/semiA;
                                            double sv = (ax*mnr[0]+ay*mnr[1]+az*mnr[2])/Rr;
                                            if (std::abs(hh) > vtol || std::abs(std::sqrt(su*su+sv*sv)-1.0)*Rr > vtol) okv = false;
                                        }
                                        if (okv) {
                                            fitC = Pc; fitE1 = maj; fitE2 = mnr; fitN = n2;
                                            fitA = semiA; fitB = Rr; fit_ok = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if (fit_ok && fitA == 0) { fitA = fitR; fitB = fitR; }
        if (fit_ok) for (auto& s : sp) {
            double wx=s.second[0]-fitC[0], wy=s.second[1]-fitC[1], wz=s.second[2]-fitC[2];
            double su = (wx*fitE1[0]+wy*fitE1[1]+wz*fitE1[2])/fitA;
            double sv = (wx*fitE2[0]+wy*fitE2[1]+wz*fitE2[2])/fitB;
            double rad = std::sqrt(su*su + sv*sv);
            if (rad > 1e-12) s.second = Point(fitC[0]+(su/rad)*fitA*fitE1[0]+(sv/rad)*fitB*fitE2[0],
                                              fitC[1]+(su/rad)*fitA*fitE1[1]+(sv/rad)*fitB*fitE2[1],
                                              fitC[2]+(su/rad)*fitA*fitE1[2]+(sv/rad)*fitB*fitE2[2]);
        }
        std::sort(sp.begin(), sp.end(), [](auto&a,auto&b){return a.first<b.first;});
        // Pave dedup (OCCT PutPavesOnCurve): the same geometric event reached through both
        // operands' lifts differs by the lift tolerance; splitting at both copies spawns a
        // micro-stub edge that can never mate (spiric tips: the torus pullback seam-splits
        // AT the tips, the plane side then counter-splits ~1e-4 away). Keep one event per
        // cluster and drop events indistinguishable from the curve ends.
        {
            double clen = 0.0;
            { Point pv = C.point_at_start();
              auto dc0 = C.domain();
              for (int k = 1; k <= 8; ++k) { Point q = C.point_at(dc0.first + (dc0.second-dc0.first)*k/8.0);
                  clen += pv.distance(q); pv = q; } }
            double pave_tol = std::max(clen * 2e-4, 1e-9);
            Point cs = C.point_at_start(), ce = C.point_at_end();
            std::vector<std::pair<double,Point>> ded;
            for (auto& s : sp) {
                if (s.second.distance(cs) < pave_tol || s.second.distance(ce) < pave_tol) continue;
                if (!ded.empty() && s.second.distance(ded.back().second) < pave_tol) continue;
                ded.push_back(s);
            }
            sp.swap(ded);
        }
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

        if (fit_ok) {
            const double PI = 3.14159265358979323846;
            double fmin = std::min(fitA, fitB);
            double rtol = std::max(1e-7 * fmin, 1e-9);
            Vector sE1(fitE1[0]*fitA, fitE1[1]*fitA, fitE1[2]*fitA);
            Vector sE2(fitE2[0]*fitB, fitE2[1]*fitB, fitE2[2]*fitB);
            auto vpos = [&](int vid) -> Point {
                int pi = (vid>=0 && vid<(int)m_topology_vertices.size()) ? m_topology_vertices[vid].point_index : -1;
                return (pi>=0 && pi<(int)m_vertices.size()) ? m_vertices[pi] : Point(0,0,0);
            };
            auto ang_of = [&](const Point& P) {
                double wx=P[0]-fitC[0], wy=P[1]-fitC[1], wz=P[2]-fitC[2];
                return std::atan2((wx*fitE2[0]+wy*fitE2[1]+wz*fitE2[2])/fitB,
                                  (wx*fitE1[0]+wy*fitE1[1]+wz*fitE1[2])/fitA);
            };
            auto off_circle = [&](const Point& P) {
                double wx=P[0]-fitC[0], wy=P[1]-fitC[1], wz=P[2]-fitC[2];
                double h = wx*fitN[0]+wy*fitN[1]+wz*fitN[2];
                double su = (wx*fitE1[0]+wy*fitE1[1]+wz*fitE1[2])/fitA;
                double sv = (wx*fitE2[0]+wy*fitE2[1]+wz*fitE2[2])/fitB;
                return std::max(std::abs(h), std::abs(std::sqrt(su*su+sv*sv) - 1.0)*fmin);
            };
            // Rebuild `poly`'s span as the exact arc from A to B; the polyline supplies the winding
            // (which way round, how many quarters) - its endpoint gap only fixes swept mod 2*pi.
            auto rebuild = [&](const NurbsCurve& poly, const Point& A, const Point& B, NurbsCurve& out) {
                if (off_circle(A) > rtol || off_circle(B) > rtol) return false;
                double sw_poly = 0.0, prev = 0.0; bool first = true;
                for (int i = 0; i < poly.cv_count(); ++i) {
                    double a = ang_of(poly.get_cv(i));
                    if (!first) { double d = a - prev;
                        while (d >  PI) d -= 2*PI; while (d < -PI) d += 2*PI; sw_poly += d; }
                    prev = a; first = false;
                }
                double a0 = ang_of(A), a1 = ang_of(B);
                double sw = (a1 - a0) + 2*PI*std::round((sw_poly - (a1 - a0)) / (2*PI));
                if (std::abs(sw) < 1e-9 || std::abs(sw) > 2*PI + 1e-9) return false;
                out = exact_arc_3d(fitC, sE1, sE2, 1.0, a0, sw);
                return out.is_valid();
            };
            for (size_t k = 0; k < arcs.size(); ++k) {
                NurbsCurve na;
                if (rebuild(arcs[k], vpos(arc_v[k][0]), vpos(arc_v[k][1]), na)) arcs[k] = na;
            }
            // Propagate to the coincident opposite-operand arcs (co-oriented, same snapped
            // endpoints -> identical geometry on both sides): sew keeps the earliest edge as
            // representative, so BOTH candidates must carry the exact arc for it to survive.
            for (int ej = 0; ej < ne0; ++ej) {
                if (ej == ei || !cand(ej) || bbox_far(ei,ej) || !subset_of(ej,ei)) continue;
                int cj = m_topology_edges[ej].curve_3d_index;
                const NurbsCurve& Cj = m_curves_3d[cj];
                Point ja = Cj.point_at_start(), jb = Cj.point_at_end();
                if (ja.distance(jb) < tol) continue;
                for (size_t k = 0; k < arcs.size(); ++k) {
                    Point A = vpos(arc_v[k][0]), B = vpos(arc_v[k][1]);
                    bool fw = ja.distance(A) < tol && jb.distance(B) < tol;
                    bool bw = ja.distance(B) < tol && jb.distance(A) < tol;
                    if (!fw && !bw) continue;
                    NurbsCurve na;
                    if (rebuild(Cj, fw ? A : B, fw ? B : A, na)) m_curves_3d[cj] = na;
                    break;
                }
            }
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
                // Try splitting the ORIGINAL pcurve first for EVERY face: it carries the
                // section's fit accuracy, while the 3D arcs are devtol-coarse polyline lifts
                // (the planar affine rebuild inherits their sag: the split spiric discs lost
                // 0.7% area through it). The one known failure -- NurbsCurve::split on a
                // CLOSED rational circle -- is pre-gated; any other failure falls through to
                // the planar affine-from-arcs rebuild.
                bool closed_rational = false;
                if (c2>=0 && c2<(int)m_curves_2d.size()) {
                    const NurbsCurve& P0 = m_curves_2d[c2];
                    auto pd0 = P0.domain();
                    closed_rational = P0.is_rational()
                        && P0.point_at(pd0.first).distance(P0.point_at(pd0.second)) < 1e-12;
                }
                if (c2>=0 && c2<(int)m_curves_2d.size() && !closed_rational) {
                    NurbsCurve P=m_curves_2d[c2];
                    // Split param via the SURFACE-MAPPED pcurve, not Closest::surface_point:
                    // near a periodic seam Closest clamps u to the principal branch boundary,
                    // closest_parameter then lands at a domain end, the split fails or lands
                    // 1e-2 off, and the ok=false path below binds the WHOLE pcurve to piece 0
                    // (the cross-assigned-edge corruption). Projecting the 3D split point onto
                    // the pcurve's own 3D image is branch-free and exact for polyline pcurves.
                    auto param_near_3d = [&](const NurbsCurve& P2, const NurbsSurface& S2, const Point& V) -> double {
                        auto pd = P2.domain(); int M = std::min(std::max(64, P2.cv_count()*4), 4096);
                        double bt = pd.first, bd = 1e300; Point prev3(0,0,0); double prevt = pd.first;
                        for (int i2 = 0; i2 <= M; ++i2) {
                            double t = pd.first + (pd.second - pd.first)*i2/M;
                            Point uv = P2.point_at(t); Point q = S2.point_at(uv[0], uv[1]);
                            if (i2 > 0) {
                                double ex=q[0]-prev3[0], ey=q[1]-prev3[1], ez=q[2]-prev3[2];
                                double L2=ex*ex+ey*ey+ez*ez;
                                double lam=(L2>1e-30)?((V[0]-prev3[0])*ex+(V[1]-prev3[1])*ey+(V[2]-prev3[2])*ez)/L2:0.0;
                                lam=std::min(std::max(lam,0.0),1.0);
                                double cx=prev3[0]+lam*ex, cy=prev3[1]+lam*ey, cz=prev3[2]+lam*ez;
                                double d=(V[0]-cx)*(V[0]-cx)+(V[1]-cy)*(V[1]-cy)+(V[2]-cz)*(V[2]-cz);
                                if (d<bd){ bd=d; bt=prevt+lam*(t-prevt); }
                            }
                            prev3=q; prevt=t;
                        }
                        auto d3at = [&](double t){ Point uv=P2.point_at(t); Point q=S2.point_at(uv[0],uv[1]);
                            double dx=q[0]-V[0], dy=q[1]-V[1], dz=q[2]-V[2]; return dx*dx+dy*dy+dz*dz; };
                        double lo = std::max(pd.first,  bt - (pd.second-pd.first)/M);
                        double hi = std::min(pd.second, bt + (pd.second-pd.first)/M);
                        for (int it = 0; it < 80 && hi-lo > 1e-14*(pd.second-pd.first); ++it) {
                            double m1 = lo + (hi-lo)/3, m2 = hi - (hi-lo)/3;
                            if (d3at(m1) < d3at(m2)) hi = m2; else lo = m1;
                        }
                        return 0.5*(lo+hi);
                    };
                    std::vector<double> tps;
                    for (const Point& V : ipts) tps.push_back(param_near_3d(P, S, V));
                    p2 = split_multi(P, tps);
                    if (std::getenv("SESSION_COREF_DBG")) {
                        std::fprintf(stderr, "[COREF] ei=%d ti=%d split-orig p2=%d want=%d dom=[%.6f,%.6f] tps0=%.9f nk=%d cv=%d\n",
                            ei, ti, (int)p2.size(), (int)edge_ids.size(),
                            P.domain().first, P.domain().second, tps.empty()?-1.0:tps[0],
                            P.nurbsknot_count(), P.cv_count());
                        if (!p2.empty() && p2[0].is_valid() && !tps.empty()) {
                            Point pj = p2[0].get_cv(p2[0].cv_count()-1);
                            Point pe = P.point_at(tps[0]);
                            std::fprintf(stderr, "[COREF]   junction cv=(%.9f,%.9f) P.at(tps0)=(%.9f,%.9f)\n",
                                pj[0], pj[1], pe[0], pe[1]);
                            int nP = P.cv_count();
                            for (int qq = 0; qq < nP; ++qq) {
                                Point pc0 = P.get_cv(qq);
                                Point pn0 = P.get_cv(std::min(qq+1, nP-1));
                                if ((pc0[1]-0.5)*(pn0[1]-0.5) <= 0.0 && std::abs(pc0[0]) < 0.1)
                                    std::fprintf(stderr, "[COREF]   P.cv[%d]=(%.9f,%.9f) cv[%d]=(%.9f,%.9f)\n",
                                        qq, pc0[0], pc0[1], qq+1, pn0[0], pn0[1]);
                            }
                        }
                    }
                    if (do_wrap && (int)p2.size()>=2){ auto w2=NurbsCurve::join({p2.back(),p2.front()}, tol);
                        if(w2.size()==1 && w2[0].is_valid()){ std::vector<NurbsCurve> a2(p2.begin()+1,p2.end()-1);
                            a2.push_back(w2[0]); p2=a2; } else p2.clear(); }
                }
                if ((int)p2.size() != (int)edge_ids.size() && S.is_planar(nullptr, 1e-6)) {
                    if (std::getenv("SESSION_COREF_DBG"))
                        std::fprintf(stderr, "[COREF] ei=%d ti=%d AFFINE fallback\n", ei, ti);
                    p2.clear();
                    // Planar face: for a parallelogram-CV degree-1x1 patch (every box/plane face)
                    // the UV<->3D map is AFFINE, which preserves a rational NURBS with its control
                    // points mapped and weights unchanged -> an EXACT rational-arc pcurve. The mid
                    // CVs of >=90-deg arcs lie OUTSIDE the patch, where Closest::surface_point would
                    // clamp to the domain and corrupt the arc, so invert the affine frame directly;
                    // Closest remains the per-CV fallback for non-affine planar patches.
                    auto [su0,su1] = S.domain(0); auto [sv0,sv1] = S.domain(1);
                    Point O3 = S.point_at(su0, sv0);
                    Point PU = S.point_at(su1, sv0), PV = S.point_at(su0, sv1), PW = S.point_at(su1, sv1);
                    double dux=(PU[0]-O3[0])/(su1-su0), duy=(PU[1]-O3[1])/(su1-su0), duz=(PU[2]-O3[2])/(su1-su0);
                    double dvx=(PV[0]-O3[0])/(sv1-sv0), dvy=(PV[1]-O3[1])/(sv1-sv0), dvz=(PV[2]-O3[2])/(sv1-sv0);
                    double guu=dux*dux+duy*duy+duz*duz, guv=dux*dvx+duy*dvy+duz*dvz, gvv=dvx*dvx+dvy*dvy+dvz*dvz;
                    double gdet = guu*gvv - guv*guv;
                    double psz = O3.distance(PW);
                    bool affine = S.degree(0)==1 && S.degree(1)==1 && gdet > 1e-20
                        && std::abs(PW[0]-PU[0]-PV[0]+O3[0]) + std::abs(PW[1]-PU[1]-PV[1]+O3[1])
                         + std::abs(PW[2]-PU[2]-PV[2]+O3[2]) < psz*1e-9 + 1e-12;
                    auto to_uv = [&](const Point& P, double& u, double& v) {
                        double wx=P[0]-O3[0], wy=P[1]-O3[1], wz=P[2]-O3[2];
                        double bu=wx*dux+wy*duy+wz*duz, bv=wx*dvx+wy*dvy+wz*dvz;
                        u = su0 + (bu*gvv - bv*guv)/gdet;
                        v = sv0 + (guu*bv - guv*bu)/gdet;
                    };
                    for (const NurbsCurve& arc3 : arcs) {
                        NurbsCurve pc; int ord=arc3.order(), nc=arc3.cv_count();
                        bool ok2 = nc>=2 && pc.create(3, arc3.is_rational(), ord, nc)
                                          && pc.nurbsknot_count()==arc3.nurbsknot_count();
                        for (int i=0;i<nc && ok2;++i){ Point Pe=arc3.get_cv(i); double w=arc3.weight(i);
                            double uu, vv;
                            if (affine) to_uv(Pe, uu, vv);
                            else { auto [au,av,dd]=Closest::surface_point(S,Pe); (void)dd; uu=au; vv=av; }
                            ok2 = pc.set_cv_4d(i, uu*w, vv*w, 0.0, w); }
                        for (int k=0;k<arc3.nurbsknot_count() && ok2;++k) ok2 = pc.set_nurbsknot(k, arc3.nurbsknot(k));
                        if (ok2 && pc.is_valid()) { p2.push_back(pc); continue; }
                        auto ad = arc3.domain(); int n = std::min(std::max(arc3.cv_count()*2, 12), 2048);
                        std::vector<Point> uvs;
                        for (int s=0;s<=n;++s){ Point P3=arc3.point_at(ad.first+(ad.second-ad.first)*s/n);
                            auto [uu,vv,dd]=Closest::surface_point(S,P3); (void)dd; uvs.push_back(Point(uu,vv,0.0)); }
                        p2.push_back(NurbsCurve::create(false,1,uvs));
                    }

                }
            }
            bool ok = ((int)p2.size()==(int)edge_ids.size());
            // A MATE-oriented trim's pcurve runs opposite the shared 3D curve (the edge was lifted
            // from the OTHER face's pcurve), so pieces in pcurve-param order do not line up
            // index-for-index with the 3D arcs -- index pairing hands each trim a DIFFERENT arc of
            // the circle. Keep the pieces in pcurve/loop order (meshing chains them in sequence)
            // and instead match each piece to ITS edge geometrically (endpoint-compatible,
            // midpoint tie-break across same-endpoint halves).
            std::vector<int> edge_for(edge_ids.begin(), edge_ids.end());
            if (ok && si>=0 && si<(int)m_surfaces.size()) {
                const NurbsSurface& S2 = m_surfaces[si];
                std::vector<int> ef(p2.size(), -1);
                std::vector<bool> taken(arcs.size(), false);
                bool allm = true;
                for (size_t j = 0; j < p2.size() && allm; ++j) {
                    if (!p2[j].is_valid()) { allm = false; break; }
                    auto pd = p2[j].domain();
                    Point a2u = p2[j].point_at(pd.first), b2u = p2[j].point_at(pd.second);
                    Point m2u = p2[j].point_at(0.5*(pd.first+pd.second));
                    Point a3 = S2.point_at(a2u[0],a2u[1]), b3 = S2.point_at(b2u[0],b2u[1]);
                    Point m3 = S2.point_at(m2u[0],m2u[1]);
                    int hit = -1; double bestm = 1e300;
                    for (size_t k = 0; k < arcs.size(); ++k) {
                        if (taken[k]) continue;
                        Point ea = arcs[k].point_at_start(), eb = arcs[k].point_at_end();
                        bool epok = (a3.distance(ea) < tol && b3.distance(eb) < tol)
                                 || (a3.distance(eb) < tol && b3.distance(ea) < tol);
                        if (!epok) continue;
                        auto ad2 = arcs[k].domain();
                        Point em = arcs[k].point_at(0.5*(ad2.first+ad2.second));
                        double dm = m3.distance(em);
                        if (dm < bestm) { bestm = dm; hit = (int)k; }
                    }
                    if (hit < 0) { allm = false; break; }
                    taken[hit] = true; ef[j] = edge_ids[hit];
                }
                if (allm) edge_for.assign(ef.begin(), ef.end());
            }
            std::vector<int> newtrims;
            for (size_t k=0;k<edge_ids.size();++k){
                int c2idx,tidx;
                NurbsCurve pc = ok ? p2[k] : (k==0 ? m_curves_2d[std::max(c2,0)] : NurbsCurve());
                if(k==0){ c2idx=(c2>=0?c2:add_curve_2d(pc)); if(c2>=0)m_curves_2d[c2]=pc; tidx=ti; }
                else { c2idx=add_curve_2d(pc); tidx=(int)m_trims.size(); m_trims.push_back(BRepTrim()); }
                m_trims[tidx].curve_2d_index=c2idx; m_trims[tidx].edge_index=edge_for[k];
                m_trims[tidx].loop_index=li; m_trims[tidx].reversed=T.reversed; m_trims[tidx].type=T.type;
                newtrims.push_back(tidx); m_topology_edges[edge_for[k]].trim_indices.push_back(tidx);
            }
            if (T.reversed) std::reverse(newtrims.begin(), newtrims.end());
            if (li>=0 && li<(int)m_loops.size()){ auto& tl=m_loops[li].trim_indices;
                auto it=std::find(tl.begin(),tl.end(),ti);
                if(it!=tl.end()){ int pos=(int)(it-tl.begin()); tl.erase(it); tl.insert(tl.begin()+pos,newtrims.begin(),newtrims.end()); } }
        }
    }
    // FINAL PASS: upgrade every UNSPLIT closed section circle (a densely-lifted degree-1 polyline
    // whose vertices verifiably lie on one circle) to the exact rational circle, phase-anchored at
    // its start CV and co-wound with the polyline so trim pcurve<->c3d endpoint correspondence is
    // preserved. The polyline inscribes the circle, so the sphere/torus boundary-integral flux was
    // ~0.6% short per cap loop (box x sphere: the three unsplit cap circles were the whole residual);
    // denser sampling cannot fix wrong geometry. Split circles are handled arc-by-arc above.
    {
        const double PI = 3.14159265358979323846;
        for (auto& e : m_topology_edges) {
            int ci = e.curve_3d_index;
            if (ci < 0 || ci >= (int)m_curves_3d.size()) continue;
            const NurbsCurve& pl = m_curves_3d[ci];
            if (!pl.is_valid() || pl.degree() != 1 || pl.is_rational() || pl.cv_count() < 12) continue;
            if (pl.point_at_start().distance(pl.point_at_end()) > 1e-9) continue;
            std::vector<Point> cpts;
            for (int i = 0; i < pl.cv_count(); ++i) cpts.push_back(pl.get_cv(i));
            Point fc; Vector fn, f1, f2; double fr = 0;
            if (!circle_through_points(cpts, fc, fn, f1, f2, fr)) continue;
            double sw = 0.0, prev = 0.0; bool first = true;
            for (const auto& p : cpts) {
                double a = std::atan2((p[0]-fc[0])*f2[0]+(p[1]-fc[1])*f2[1]+(p[2]-fc[2])*f2[2],
                                      (p[0]-fc[0])*f1[0]+(p[1]-fc[1])*f1[1]+(p[2]-fc[2])*f1[2]);
                if (!first) { double d = a - prev;
                    while (d >  PI) d -= 2*PI; while (d < -PI) d += 2*PI; sw += d; }
                prev = a; first = false;
            }
            if (std::abs(std::abs(sw) - 2*PI) > 1e-6) continue;
            NurbsCurve ex = exact_arc_3d(fc, f1, f2, fr, 0.0, sw > 0 ? 2*PI : -2*PI);
            if (ex.is_valid()) m_curves_3d[ci] = ex;
        }
    }
    // FINAL PASS 2: coincident PAIRS of under-mated OPEN polyline arcs whose two adjacent
    // surfaces form a recognized quadric pair carry an exactly-derivable section conic
    // (sphere-sphere radical circle; equal-radius crossing cylinders' Steinmetz ellipse).
    // Both sides' lift polylines inscribe it (~1e-4); rebuild BOTH as the exact rational arc
    // through the SAME snapped endpoints so they are bit-identical for sew and the volume
    // integrals. (Split arcs are handled inside the split loop above; this covers the
    // 1:1-mating imprints that never split, e.g. cyl x cyl.)
    {
        const double PI = 3.14159265358979323846;
        double ptol = 5e-3 * 1.0; // endpoint pairing tolerance, refined below from the diag
        {
            double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
            for (const auto& q : m_vertices) { xmn=std::min(xmn,q[0]); ymn=std::min(ymn,q[1]); zmn=std::min(zmn,q[2]);
                xmx=std::max(xmx,q[0]); ymx=std::max(ymx,q[1]); zmx=std::max(zmx,q[2]); }
            double dg = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
            if (dg > 0) ptol = 5e-3 * dg;
        }
        auto srf_of = [&](int e) -> int {
            for (int t : m_topology_edges[e].trim_indices) {
                if (t < 0 || t >= (int)m_trims.size()) continue;
                int l = m_trims[t].loop_index; if (l < 0 || l >= (int)m_loops.size()) continue;
                int f = m_loops[l].face_index; if (f < 0 || f >= (int)m_faces.size()) continue;
                return m_faces[f].surface_index;
            }
            return -1;
        };
        auto arcish_n = [&](int e, int want_trims) -> const NurbsCurve* {
            if ((int)m_topology_edges[e].trim_indices.size() != want_trims) return nullptr;
            int c = m_topology_edges[e].curve_3d_index;
            if (c < 0 || c >= (int)m_curves_3d.size()) return nullptr;
            const NurbsCurve& cv = m_curves_3d[c];
            if (!cv.is_valid() || cv.degree() != 1 || cv.is_rational() || cv.cv_count() < 8) return nullptr;
            if (cv.point_at_start().distance(cv.point_at_end()) < 1e-9) return nullptr;   // closed: pass 1
            return &cv;
        };
        auto arcish = [&](int e){ return arcish_n(e, 1); };
        // surface indices of ALL distinct faces adjacent to an edge (for the 2-trim case)
        auto srfs_of = [&](int e, int& o1, int& o2) {
            o1 = -1; o2 = -1;
            for (int t : m_topology_edges[e].trim_indices) {
                if (t < 0 || t >= (int)m_trims.size()) continue;
                int l = m_trims[t].loop_index; if (l < 0 || l >= (int)m_loops.size()) continue;
                int f = m_loops[l].face_index; if (f < 0 || f >= (int)m_faces.size()) continue;
                int si2 = m_faces[f].surface_index;
                if (o1 < 0) o1 = si2; else if (si2 != o1 && o2 < 0) o2 = si2;
            }
        };
        int ne2 = (int)m_topology_edges.size();
        // SameParameter guard (OCCT invariant): an edge whose 3D curve does not geometrically
        // span what its trims' pcurves span (a cross-assigned or stale curve, e.g. a sub-tol
        // sliver left at a near-duplicate split event) poisons every consumer -- the volume
        // c3 gate falls back to the sagging pcurve lift, sew mismatches, meshing tears.
        // Detect by endpoint+midpoint mismatch against the first trim's surface-mapped pcurve
        // and re-lift the 3D curve from that pcurve; the conic sweep below then upgrades the
        // re-lift to the exact arc whenever it inscribes a registered conic.
        int n_relift = 0;
        for (int e2 = 0; e2 < ne2; ++e2) {
            if (m_topology_edges[e2].trim_indices.empty()) continue;
            int c = m_topology_edges[e2].curve_3d_index;
            if (c < 0 || c >= (int)m_curves_3d.size()) continue;
            const NurbsCurve& cv = m_curves_3d[c];
            if (!cv.is_valid()) continue;
            int ti = m_topology_edges[e2].trim_indices[0];
            if (ti < 0 || ti >= (int)m_trims.size()) continue;
            int pci = m_trims[ti].curve_2d_index;
            int l = m_trims[ti].loop_index; if (l < 0 || l >= (int)m_loops.size()) continue;
            int f = m_loops[l].face_index; if (f < 0 || f >= (int)m_faces.size()) continue;
            int si = m_faces[f].surface_index;
            if (pci < 0 || pci >= (int)m_curves_2d.size() || si < 0 || si >= (int)m_surfaces.size()) continue;
            const NurbsCurve& pc = m_curves_2d[pci];
            const NurbsSurface& srf = m_surfaces[si];
            auto dc = pc.domain();
            Point uv0 = pc.point_at(dc.first), uv1 = pc.point_at(dc.second), uvm = pc.point_at(0.5*(dc.first+dc.second));
            Point q0 = srf.point_at(uv0[0], uv0[1]), q1 = srf.point_at(uv1[0], uv1[1]), qm = srf.point_at(uvm[0], uvm[1]);
            auto d3 = cv.domain();
            Point ca = cv.point_at(d3.first), cb = cv.point_at(d3.second);
            double dfw = std::max(q0.distance(ca), q1.distance(cb));
            double dbw = std::max(q0.distance(cb), q1.distance(ca));
            double dm = 1e300, dmt = d3.first;
            const int MK = 128;
            for (int k = 0; k <= MK; ++k) {
                double t = d3.first + (d3.second-d3.first)*k/MK;
                double d = qm.distance(cv.point_at(t));
                if (d < dm) { dm = d; dmt = t; }
            }
            {   // refine: coarse samples on a long curve overestimate by half the spacing,
                // which false-flags exact circles as mismatched
                double lo = std::max(d3.first,  dmt - (d3.second-d3.first)/MK);
                double hi = std::min(d3.second, dmt + (d3.second-d3.first)/MK);
                for (int it = 0; it < 60 && hi-lo > 1e-14*(d3.second-d3.first); ++it) {
                    double m1 = lo + (hi-lo)/3, m2 = hi - (hi-lo)/3;
                    if (qm.distance(cv.point_at(m1)) < qm.distance(cv.point_at(m2))) hi = m2; else lo = m1;
                }
                dm = std::min(dm, qm.distance(cv.point_at(0.5*(lo+hi))));
            }
            if (std::min(dfw, dbw) < ptol && dm < ptol) continue;   // curve spans the trim: OK
            int NL = 64;
            std::vector<Point> lp; lp.reserve(NL+1);
            for (int k = 0; k <= NL; ++k) {
                Point uv = pc.point_at(dc.first + (dc.second-dc.first)*k/NL);
                lp.push_back(srf.point_at(uv[0], uv[1]));
            }
            NurbsCurve nl = NurbsCurve::create(false, 1, lp);
            if (!nl.is_valid()) continue;
            m_curves_3d.push_back(nl);
            m_topology_edges[e2].curve_3d_index = (int)m_curves_3d.size() - 1;
            ++n_relift;
        }
        // Conic registry: derive each surface-pair section conic ONCE (from any coincident arc
        // pair or any 2-trim arc), then upgrade EVERY inscribed polyline arc through its OWN
        // snapped endpoints. Forcing a pair onto shared endpoints is wrong when the two sides
        // were split at different events (A's seam vs B's seam, ~1e-2 apart): each face's
        // boundary must pass through ITS OWN junctions; the pave engine mates the sides later.
        struct ConicRec { Point C; Vector E1, E2, N; double A, B; };
        std::vector<ConicRec> conics;
        auto reg_conic = [&](const Point& C2, const Vector& E1, const Vector& E2, const Vector& N2, double A2, double B2) {
            for (const auto& g : conics)
                if (g.C.distance(C2) < 1e-9 && std::abs(g.A-A2) < 1e-9 && std::abs(g.B-B2) < 1e-9
                    && std::abs(g.N[0]*N2[0]+g.N[1]*N2[1]+g.N[2]*N2[2]) > 1.0-1e-9) return;
            conics.push_back({C2, E1, E2, N2, A2, B2});
        };
        // sphere-sphere radical circle / equal-radius crossing cylinders' BOTH Steinmetz
        // ellipses; the fit test in the sweep below assigns each arc to its conic.
        auto derive_conics = [&](int s1, int s2) -> bool {
            bool got = false;
            Point C1, C2s; double R1 = 0, R2 = 0;
            if (sphere_of_surface(m_surfaces[s1], C1, R1) && sphere_of_surface(m_surfaces[s2], C2s, R2)) {
                double dx=C2s[0]-C1[0], dy=C2s[1]-C1[1], dz=C2s[2]-C1[2];
                double d = std::sqrt(dx*dx+dy*dy+dz*dz);
                if (d > 1e-9 && d < R1+R2) {
                    double a = (d*d + R1*R1 - R2*R2) / (2.0*d);
                    double rr2 = R1*R1 - a*a;
                    if (rr2 > 1e-18) {
                        Vector fN(dx/d, dy/d, dz/d);
                        Point fC(C1[0]+a*fN[0], C1[1]+a*fN[1], C1[2]+a*fN[2]);
                        double fR = std::sqrt(rr2);
                        int k = std::abs(fN[0]) <= std::abs(fN[1])
                              ? (std::abs(fN[0]) <= std::abs(fN[2]) ? 0 : 2)
                              : (std::abs(fN[1]) <= std::abs(fN[2]) ? 1 : 2);
                        Vector ax(k==0?1.0:0.0, k==1?1.0:0.0, k==2?1.0:0.0);
                        double h = ax[0]*fN[0]+ax[1]*fN[1]+ax[2]*fN[2];
                        double px=ax[0]-h*fN[0], py=ax[1]-h*fN[1], pz=ax[2]-h*fN[2];
                        double pl = std::sqrt(px*px+py*py+pz*pz);
                        if (pl > 1e-12) {
                            Vector fE1(px/pl, py/pl, pz/pl);
                            Vector fE2(fN[1]*fE1[2]-fN[2]*fE1[1], fN[2]*fE1[0]-fN[0]*fE1[2], fN[0]*fE1[1]-fN[1]*fE1[0]);
                            reg_conic(fC, fE1, fE2, fN, fR, fR);
                            got = true;
                        }
                    }
                }
                return got;
            }
            Point A1, A2; Vector W1, W2; double Rc1 = 0, Rc2 = 0;
            if (cylinder_of_surface(m_surfaces[s1], A1, W1, Rc1)
             && cylinder_of_surface(m_surfaces[s2], A2, W2, Rc2)
             && std::abs(Rc1-Rc2) < 1e-6*std::max(Rc1,Rc2)) {
                double b = W1[0]*W2[0]+W1[1]*W2[1]+W1[2]*W2[2];
                double den = 1.0 - b*b;
                if (den > 1e-12) {
                    double d0x=A2[0]-A1[0], d0y=A2[1]-A1[1], d0z=A2[2]-A1[2];
                    double dw1 = d0x*W1[0]+d0y*W1[1]+d0z*W1[2];
                    double dw2 = d0x*W2[0]+d0y*W2[1]+d0z*W2[2];
                    double t1 = (dw1 - b*dw2)/den, t2 = (b*dw1 - dw2)/den;
                    Point q1(A1[0]+t1*W1[0], A1[1]+t1*W1[1], A1[2]+t1*W1[2]);
                    Point q2(A2[0]+t2*W2[0], A2[1]+t2*W2[1], A2[2]+t2*W2[2]);
                    if (q1.distance(q2) < 1e-6) {
                        double Rr = 0.5*(Rc1+Rc2);
                        double cx=W1[1]*W2[2]-W1[2]*W2[1], cy=W1[2]*W2[0]-W1[0]*W2[2], cz=W1[0]*W2[1]-W1[1]*W2[0];
                        double cl = std::sqrt(cx*cx+cy*cy+cz*cz);
                        double ang = std::atan2(cl, b);
                        double sh = std::sin(0.5*ang), ch = std::cos(0.5*ang);
                        if (cl > 1e-9 && sh > 1e-9 && ch > 1e-9) {
                            Vector mnr(cx/cl, cy/cl, cz/cl);
                            Point Pc(0.5*(q1[0]+q2[0]), 0.5*(q1[1]+q2[1]), 0.5*(q1[2]+q2[2]));
                            for (int cand = 0; cand < 2; ++cand) {
                                double sx = cand==0 ? 1.0 : -1.0;
                                double mx=W1[0]+sx*W2[0], my=W1[1]+sx*W2[1], mz=W1[2]+sx*W2[2];
                                double ml = std::sqrt(mx*mx+my*my+mz*mz);
                                if (ml < 1e-9) continue;
                                Vector maj(mx/ml, my/ml, mz/ml);
                                double semiA = cand==0 ? Rr/sh : Rr/ch;
                                Vector n2(maj[1]*mnr[2]-maj[2]*mnr[1], maj[2]*mnr[0]-maj[0]*mnr[2], maj[0]*mnr[1]-maj[1]*mnr[0]);
                                reg_conic(Pc, maj, mnr, n2, semiA, Rr);
                                got = true;
                            }
                        }
                    }
                }
            }
            return got;
        };
        int n_arc1 = 0, n_arc2 = 0, n_up = 0;
        for (int ea = 0; ea < ne2; ++ea) {
            const NurbsCurve* Ca = arcish(ea); if (!Ca) continue; ++n_arc1;
            Point a0 = Ca->point_at_start(), a1 = Ca->point_at_end();
            Point am = Ca->point_at(0.5*(Ca->domain().first+Ca->domain().second));
            for (int eb = ea+1; eb < ne2; ++eb) {
                const NurbsCurve* Cb = arcish(eb); if (!Cb) continue;
                Point b0 = Cb->point_at_start(), b1 = Cb->point_at_end();
                bool fw2 = a0.distance(b0) < ptol && a1.distance(b1) < ptol;
                bool bw2 = a0.distance(b1) < ptol && a1.distance(b0) < ptol;
                if (!fw2 && !bw2) continue;
                Point bm = Cb->point_at(0.5*(Cb->domain().first+Cb->domain().second));
                double chord = a0.distance(a1) + 1e-12;
                if (am.distance(bm) > 0.35*chord + ptol) continue;   // opposite halves share ends
                int s1 = srf_of(ea), s2 = srf_of(eb);
                if (s1 < 0 || s2 < 0 || s1 == s2) continue;
                if (s1 >= (int)m_surfaces.size() || s2 >= (int)m_surfaces.size()) continue;
                if (derive_conics(s1, s2)) break;
            }
        }
        for (int ea = 0; ea < ne2; ++ea) {
            const NurbsCurve* Ca = arcish_n(ea, 2); if (!Ca) continue; ++n_arc2;
            int s1, s2; srfs_of(ea, s1, s2);
            if (s1 < 0 || s2 < 0 || s1 == s2) continue;
            if (s1 >= (int)m_surfaces.size() || s2 >= (int)m_surfaces.size()) continue;
            derive_conics(s1, s2);
        }
        // upgrade sweep: every open polyline arc inscribing a registered conic is rebuilt as
        // the exact rational arc through its OWN radially-snapped endpoints (crossings exact).
        int n_closed = 0;
        for (int ea = 0; ea < ne2 && !conics.empty(); ++ea) {
            const NurbsCurve* Ca = nullptr;
            {
                int nt = (int)m_topology_edges[ea].trim_indices.size();
                int c = m_topology_edges[ea].curve_3d_index;
                if (nt >= 1 && nt <= 2 && c >= 0 && c < (int)m_curves_3d.size()) {
                    const NurbsCurve& cv = m_curves_3d[c];
                    if (cv.is_valid() && cv.degree() == 1 && !cv.is_rational() && cv.cv_count() >= 8)
                        Ca = &cv;
                }
            }
            if (!Ca) continue;
            bool closed = Ca->point_at_start().distance(Ca->point_at_end()) < 1e-9;
            if (closed) ++n_closed;
            for (const auto& g : conics) {
                double mnAB = std::min(g.A, g.B);
                double vtol2 = std::max(5e-3*mnAB, 1e-6);
                bool okv = true;
                for (int ii = 0; ii < Ca->cv_count() && okv; ++ii) {
                    Point q = Ca->get_cv(ii);
                    double ax=q[0]-g.C[0], ay=q[1]-g.C[1], az=q[2]-g.C[2];
                    double hh = ax*g.N[0]+ay*g.N[1]+az*g.N[2];
                    double su = (ax*g.E1[0]+ay*g.E1[1]+az*g.E1[2])/g.A;
                    double sv = (ax*g.E2[0]+ay*g.E2[1]+az*g.E2[2])/g.B;
                    if (std::abs(hh) > vtol2 || std::abs(std::sqrt(su*su+sv*sv)-1.0)*mnAB > vtol2) okv = false;
                }
                if (!okv) continue;
                // Steinmetz crossings are EXACT: both ellipses pass through C +- B*E2 (the shared
                // minor axis); endpoints near one snap THERE so adjacent arcs of the two conics
                // stay watertight. fA==fB (circle) has no true crossing.
                Point xr1(g.C[0]+g.B*g.E2[0], g.C[1]+g.B*g.E2[1], g.C[2]+g.B*g.E2[2]);
                Point xr2(g.C[0]-g.B*g.E2[0], g.C[1]-g.B*g.E2[1], g.C[2]-g.B*g.E2[2]);
                bool have_x = (g.A != g.B);
                auto snapc = [&](const Point& P) {
                    if (have_x && P.distance(xr1) < ptol) return xr1;
                    if (have_x && P.distance(xr2) < ptol) return xr2;
                    double wx=P[0]-g.C[0], wy=P[1]-g.C[1], wz=P[2]-g.C[2];
                    double su = (wx*g.E1[0]+wy*g.E1[1]+wz*g.E1[2])/g.A;
                    double sv = (wx*g.E2[0]+wy*g.E2[1]+wz*g.E2[2])/g.B;
                    double rr = std::sqrt(su*su+sv*sv);
                    if (rr < 1e-12) return P;
                    return Point(g.C[0]+(su/rr)*g.A*g.E1[0]+(sv/rr)*g.B*g.E2[0],
                                 g.C[1]+(su/rr)*g.A*g.E1[1]+(sv/rr)*g.B*g.E2[1],
                                 g.C[2]+(su/rr)*g.A*g.E1[2]+(sv/rr)*g.B*g.E2[2]);
                };
                auto ang_of2 = [&](const Point& P) {
                    double wx=P[0]-g.C[0], wy=P[1]-g.C[1], wz=P[2]-g.C[2];
                    return std::atan2((wx*g.E2[0]+wy*g.E2[1]+wz*g.E2[2])/g.B,
                                      (wx*g.E1[0]+wy*g.E1[1]+wz*g.E1[2])/g.A);
                };
                Point SA = snapc(Ca->point_at_start()), SB = snapc(Ca->point_at_end());
                double sw_poly = 0.0, prev = 0.0; bool first2 = true;
                for (int ii = 0; ii < Ca->cv_count(); ++ii) {
                    double a = ang_of2(Ca->get_cv(ii));
                    if (!first2) { double d = a - prev;
                        while (d >  PI) d -= 2*PI; while (d < -PI) d += 2*PI; sw_poly += d; }
                    prev = a; first2 = false;
                }
                double aa0 = ang_of2(SA), aa1 = ang_of2(SB);
                double sw;
                if (closed) {
                    if (std::abs(std::abs(sw_poly) - 2*PI) > 0.1) break;   // not a simple loop
                    sw = sw_poly >= 0 ? 2*PI : -2*PI;
                } else {
                    sw = (aa1 - aa0) + 2*PI*std::round((sw_poly - (aa1 - aa0)) / (2*PI));
                }
                if (std::abs(sw) < 1e-9 || std::abs(sw) > 2*PI + 1e-9) break;
                Vector sE1(g.E1[0]*g.A, g.E1[1]*g.A, g.E1[2]*g.A);
                Vector sE2(g.E2[0]*g.B, g.E2[1]*g.B, g.E2[2]*g.B);
                NurbsCurve na2 = exact_arc_3d(g.C, sE1, sE2, 1.0, aa0, sw);
                if (na2.is_valid()) { m_curves_3d[m_topology_edges[ea].curve_3d_index] = na2; ++n_up; }
                break;
            }
        }
        if (std::getenv("SESSION_XCHK")) {
            std::fprintf(stderr, "[PAIRPASS] arc1=%d arc2=%d conics=%d up=%d closed=%d relift=%d\n", n_arc1, n_arc2, (int)conics.size(), n_up, n_closed, n_relift);
            for (int e = 0; e < ne2; ++e) {
                int c = m_topology_edges[e].curve_3d_index;
                if (c < 0 || c >= (int)m_curves_3d.size()) continue;
                const NurbsCurve& cv = m_curves_3d[c];
                if (!cv.is_valid() || cv.degree() != 1 || cv.is_rational()) continue;
                Point q0 = cv.point_at_start(), q1 = cv.point_at_end();
                Point qm = cv.point_at(0.5*(cv.domain().first+cv.domain().second));
                double dev = 0.0;
                { double cxx=q1[0]-q0[0], cyy=q1[1]-q0[1], czz=q1[2]-q0[2];
                  double cl2 = cxx*cxx+cyy*cyy+czz*czz;
                  for (int ii = 0; ii < cv.cv_count(); ++ii) { Point q = cv.get_cv(ii);
                    double wx=q[0]-q0[0], wy=q[1]-q0[1], wz=q[2]-q0[2];
                    double t = cl2 > 1e-24 ? (wx*cxx+wy*cyy+wz*czz)/cl2 : 0.0;
                    double dx=wx-t*cxx, dy=wy-t*cyy, dz=wz-t*czz;
                    dev = std::max(dev, std::sqrt(dx*dx+dy*dy+dz*dz)); } }
                std::fprintf(stderr, "[POLY] e=%d nt=%d cvs=%d dev=%.4f s(%.3f,%.3f,%.3f) m(%.3f,%.3f,%.3f) e(%.3f,%.3f,%.3f)\n",
                    e, (int)m_topology_edges[e].trim_indices.size(), cv.cv_count(), dev,
                    q0[0],q0[1],q0[2], qm[0],qm[1],qm[2], q1[0],q1[1],q1[2]);
            }
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
    // 16 samples sag ~0.03 at the tips of an elongated section arc (see co_refine NS note): the
    // measured Hausdorff then exceeds tol for truly-coincident arcs. Denser sampling tightens the
    // measurement only -- distinct edges stay O(diag) apart regardless of sample count.
    const int NS = 64;
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
    // Micro-edge collapse (OCCT BOPTools_AlgoTools::IsMicroEdge): an UNDER-MATED edge far
    // shorter than the sew tolerance is below the watertightness resolution -- the typical
    // source is a surface-TANGENCY point where two osculating section polylines cross twice
    // within sampling noise (tor x tor: 3e-4 slivers at the 4 tangent nodes), leaving a stub
    // no mate ever reproduces. Drop the edge and its trims; the loop's resulting endpoint
    // gap (== stub length) is far inside the tolerance every consumer already absorbs.
    std::vector<bool> dead(ne, false);
    {
        double micro_tol = tol * 0.2;
        for (int ei = 0; ei < ne; ++ei) {
            if (samp[ei].empty()) continue;                  // not a candidate (>= 2 trims)
            double len = 0.0;
            for (size_t k = 1; k < samp[ei].size(); ++k) len += samp[ei][k].distance(samp[ei][k-1]);
            if (len >= micro_tol) continue;
            // A 3D-degenerate edge with REAL UV extent is a pole/seam run (a sphere pole line
            // spans the full u-domain while its 3D curve is a point) -- legitimate topology,
            // never a sliver. Protect by trim type AND by UV extent.
            bool protected_run = false;
            for (int ti = 0; ti < (int)m_trims.size() && !protected_run; ++ti) {
                if (m_trims[ti].edge_index != ei) continue;
                if (m_trims[ti].type == BRepTrimType::Singular || m_trims[ti].type == BRepTrimType::Seam)
                    { protected_run = true; break; }
                int c2 = m_trims[ti].curve_2d_index;
                int li = m_trims[ti].loop_index;
                int fi = (li >= 0 && li < (int)m_loops.size()) ? m_loops[li].face_index : -1;
                int si = (fi >= 0 && fi < (int)m_faces.size()) ? m_faces[fi].surface_index : -1;
                if (c2 < 0 || c2 >= (int)m_curves_2d.size() || si < 0 || si >= (int)m_surfaces.size()) continue;
                const NurbsCurve& pc = m_curves_2d[c2];
                if (!pc.is_valid()) continue;
                auto pd = pc.domain();
                Point u0 = pc.point_at(pd.first), u1 = pc.point_at(pd.second);
                auto du = m_surfaces[si].domain(0); auto dv = m_surfaces[si].domain(1);
                double span = std::max(du.second - du.first, dv.second - dv.first);
                if (u0.distance(u1) > span * 1e-2) protected_run = true;
            }
            if (protected_run) continue;
            dead[ei] = true;
            for (int ti = 0; ti < (int)m_trims.size(); ++ti) {
                if (m_trims[ti].edge_index != ei) continue;
                int li = m_trims[ti].loop_index;
                if (li >= 0 && li < (int)m_loops.size()) {
                    auto& tl = m_loops[li].trim_indices;
                    tl.erase(std::remove(tl.begin(), tl.end(), ti), tl.end());
                }
                m_trims[ti].edge_index = -1;
            }
            m_topology_edges[ei].trim_indices.clear();
        }
    }
    std::vector<int> rep(ne, -1);
    std::vector<int> reps;
    for (int ei = 0; ei < ne; ++ei) {
        if (dead[ei]) continue;
        if (samp[ei].empty()) { rep[ei] = ei; reps.push_back(ei); continue; }
        for (int r : reps)
            if (!samp[r].empty() && !bbox_far(ei, r) && coincident_within(samp[ei], samp[r])) { rep[ei] = r; break; }
        if (rep[ei] < 0) { rep[ei] = ei; reps.push_back(ei); }
    }
    // Second pass, endpoint-anchored: one operand's section copy can be fit-accurate while
    // the other's is a devtol-coarse lift, pushing their Hausdorff just past the tolerance.
    // For pairs whose ENDPOINTS already agree at the base tolerance, relax the body test:
    // genuinely distinct edges that share endpoints (e.g. the two halves of a section loop)
    // diverge by O(lens height), far beyond 2.5x tol.
    {
        std::vector<int> uses(ne, 0);
        for (const auto& t : m_trims)
            if (t.edge_index >= 0 && t.edge_index < ne) uses[rep[t.edge_index]]++;
        double tol2 = tol * 2.5;
        for (int ei = 0; ei < ne; ++ei) {
            if (samp[ei].empty() || rep[ei] != ei) continue;
            if (std::getenv("SESSION_SEW_DBG") && uses[ei] == 1)
                std::fprintf(stderr, "[P2] candidate ei=%d uses=%d\n", ei, uses[ei]);
            if (uses[ei] != 1) continue;
            const Point& a0 = samp[ei].front(); const Point& a1 = samp[ei].back();
            for (int r : reps) {
                if (r == ei || samp[r].empty() || uses[r] != 1 || rep[r] != r || bbox_far(ei, r)) continue;
                const Point& b0 = samp[r].front(); const Point& b1 = samp[r].back();
                bool em = (a0.distance(b0) < tol && a1.distance(b1) < tol)
                       || (a0.distance(b1) < tol && a1.distance(b0) < tol);
                if (std::getenv("SESSION_SEW_DBG"))
                    std::fprintf(stderr, "[P2] ei=%d r=%d em=%d d00=%.4f d11=%.4f d01=%.4f d10=%.4f\n",
                        ei, r, em?1:0, a0.distance(b0), a1.distance(b1), a0.distance(b1), a1.distance(b0));
                if (!em) continue;
                bool ok2 = true;
                if (pt_to_polyline(samp[ei][samp[ei].size()/2], samp[r]) > tol2) ok2 = false;
                if (ok2) for (const auto& q : samp[ei]) if (pt_to_polyline(q, samp[r]) > tol2) { ok2 = false; break; }
                if (ok2) for (const auto& q : samp[r]) if (pt_to_polyline(q, samp[ei]) > tol2) { ok2 = false; break; }
                if (ok2) { rep[ei] = r; break; }
            }
        }
    }
    if (std::getenv("SESSION_SEW_DBG")) {
        for (int ei = 0; ei < ne; ++ei) {
            if (samp[ei].empty() || rep[ei] != ei) continue;
            for (int r : reps) {
                if (r == ei || samp[r].empty() || bbox_far(ei, r)) continue;
                double h = 0.0;
                for (const auto& q : samp[ei]) h = std::max(h, pt_to_polyline(q, samp[r]));
                for (const auto& q : samp[r]) h = std::max(h, pt_to_polyline(q, samp[ei]));
                if (h < tol * 20.0) {
                    int u1 = 0, u2 = 0;
                    for (const auto& t : m_trims) {
                        if (t.edge_index >= 0 && t.edge_index < ne && rep[t.edge_index] == ei) u1++;
                        if (t.edge_index >= 0 && t.edge_index < ne && rep[t.edge_index] == r) u2++;
                    }
                    std::fprintf(stderr, "[SEWMISS] e=%d(u=%d) r=%d(u=%d) haus=%.5f tol=%.5f\n", ei, u1, r, u2, h, tol);
                    for (int ti2 = 0; ti2 < (int)m_trims.size(); ++ti2) {
                        int oe2 = m_trims[ti2].edge_index;
                        if (oe2 < 0 || oe2 >= ne) continue;
                        if (rep[oe2] != ei && rep[oe2] != r) continue;
                        int li2 = m_trims[ti2].loop_index;
                        int fi2 = (li2 >= 0 && li2 < (int)m_loops.size()) ? m_loops[li2].face_index : -1;
                        int si2 = (fi2 >= 0 && fi2 < (int)m_faces.size()) ? m_faces[fi2].surface_index : -1;
                        int pl2 = (si2 >= 0 && si2 < (int)m_surfaces.size() && m_surfaces[si2].is_planar(nullptr, 1e-6)) ? 1 : 0;
                        std::fprintf(stderr, "  [SEWM3] rep=%d trim=%d face=%d srf=%d planar=%d\n",
                            rep[oe2], ti2, fi2, si2, pl2);
                    }
                    for (int oe = 0; oe < ne; ++oe)
                        if (oe != r && oe != ei && (rep[oe] == rep[r] || rep[oe] == ei))
                            std::fprintf(stderr, "  [SEWM2] edge %d merged into rep %d\n", oe, rep[oe]);
                }
            }
        }
    }
    // Geometry rep upgrade: within a merged group keep the DENSEST member's 3D curve (a pullback
    // lift at n~2000 CVs beats a devtol section fit at ~40 CVs by an order of magnitude in
    // on-curve accuracy). Rational (exact conic) members always win over polylines. Only the
    // curve geometry moves -- the rep edge's index, vertices and trims are untouched.
    for (int r : reps) {
        if (samp[r].empty()) continue;
        int rci = m_topology_edges[r].curve_3d_index;
        if (rci < 0 || rci >= (int)m_curves_3d.size()) continue;
        int best_ci = rci;
        auto better = [&](int a, int b) {   // is curve a better than curve b?
            const NurbsCurve& A = m_curves_3d[a]; const NurbsCurve& B = m_curves_3d[b];
            if (A.is_rational() != B.is_rational()) return A.is_rational();
            return A.cv_count() > B.cv_count();
        };
        for (int e = 0; e < ne; ++e) {
            if (rep[e] != r || e == r) continue;
            int ci = m_topology_edges[e].curve_3d_index;
            if (ci < 0 || ci >= (int)m_curves_3d.size() || !m_curves_3d[ci].is_valid()) continue;
            if (better(ci, best_ci)) best_ci = ci;
        }
        if (best_ci != rci) {
            NurbsCurve nc = m_curves_3d[best_ci];
            const NurbsCurve& rc = m_curves_3d[rci];
            if (nc.point_at_start().distance(rc.point_at_start())
              > nc.point_at_end().distance(rc.point_at_start())) nc.reverse();
            m_curves_3d[rci] = nc;
        }
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

void BRep::sameparameter_planar_pcurves() {
    // OCCT SameParameter-lite. After sew, an A<->B section edge is ONE edge with ONE 3D curve,
    // but its two trims still integrate their OWN pcurve copies, which differ at the section-fit
    // tolerance (~2e-4): the volume error is FIRST order in that mismatch (each face's flux
    // drifts independently; box x tor common failed at 1.8e-6 rel from exactly this). For the
    // PLANAR side of a mixed planar/curved section edge the fix is exact and cheap: affine-project
    // the shared 3D curve into the plane's UV -- both faces then read identical boundary geometry.
    for (auto& E : m_topology_edges) {
        if ((int)E.trim_indices.size() != 2) continue;
        int tp = -1, tq = -1, sp = -1, sq = -1;
        for (int ti : E.trim_indices) {
            if (ti < 0 || ti >= (int)m_trims.size()) continue;
            int li = m_trims[ti].loop_index; if (li < 0 || li >= (int)m_loops.size()) continue;
            int fi = m_loops[li].face_index; if (fi < 0 || fi >= (int)m_faces.size()) continue;
            int si = m_faces[fi].surface_index; if (si < 0 || si >= (int)m_surfaces.size()) continue;
            if (tp < 0) { tp = ti; sp = si; } else { tq = ti; sq = si; }
        }
        if (tp < 0 || tq < 0 || sp == sq) continue;
        bool p_planar = m_surfaces[sp].is_planar(nullptr, 1e-6);
        bool q_planar = m_surfaces[sq].is_planar(nullptr, 1e-6);
        if (p_planar == q_planar) continue;                  // need exactly one planar side
        int ti  = p_planar ? tp : tq;                        // planar trim: pcurve to rebuild
        int tic = p_planar ? tq : tp;                        // curved trim: geometry source
        const NurbsSurface& S  = m_surfaces[p_planar ? sp : sq];
        const NurbsSurface& Sc = m_surfaces[p_planar ? sq : sp];
        int c2 = m_trims[ti].curve_2d_index;
        int c2c = m_trims[tic].curve_2d_index;
        if (c2 < 0 || c2 >= (int)m_curves_2d.size()) continue;
        if (c2c < 0 || c2c >= (int)m_curves_2d.size()) continue;
        const NurbsCurve& pc = m_curves_2d[c2];
        const NurbsCurve& pcc = m_curves_2d[c2c];
        // Geometry source = the CURVED trim's pcurve lifted through ITS surface at its own
        // vertices -- byte-identical to what the curved face's boundary-flux integration walks.
        // Dense deg-1 arrangement polylines only: a 2-CV uv line (cylinder cap circles) or a
        // rational pcurve marks an exact-conic edge whose planar arc is already exact.
        if (!pc.is_valid() || !pcc.is_valid()) continue;
        if (pcc.degree() != 1 || pcc.is_rational() || pcc.cv_count() < 8) continue;
        std::vector<Point> gpts;
        gpts.reserve(pcc.cv_count());
        for (int i = 0; i < pcc.cv_count(); ++i) {
            Point uv = pcc.get_cv(i);
            gpts.push_back(Sc.point_at(uv[0], uv[1]));
        }
        // the lift must actually lie on the planar face (sew-tol scale), else this is not a
        // clean mixed section edge (e.g. a seam-adjacent sliver) -- leave it alone.
        {
            Vector pn = S.normal_at(0.5*(S.domain(0).first+S.domain(0).second),
                                    0.5*(S.domain(1).first+S.domain(1).second));
            Point p0 = S.point_at(S.domain(0).first, S.domain(1).first);
            double dmax = 0.0;
            for (const auto& g : gpts)
                dmax = std::max(dmax, std::abs((g[0]-p0[0])*pn[0]+(g[1]-p0[1])*pn[1]+(g[2]-p0[2])*pn[2]));
            double sscale = S.point_at(S.domain(0).first, S.domain(1).first)
                              .distance(S.point_at(S.domain(0).second, S.domain(1).second));
            if (dmax > std::max(sscale, 1.0) * 5e-3) continue;
        }
        // affine deg-1x1 patch check + inverse frame (same construction as co_refine's splice)
        auto [su0, su1] = S.domain(0); auto [sv0, sv1] = S.domain(1);
        Point O3 = S.point_at(su0, sv0);
        Point PU = S.point_at(su1, sv0), PV = S.point_at(su0, sv1), PW = S.point_at(su1, sv1);
        double dux=(PU[0]-O3[0])/(su1-su0), duy=(PU[1]-O3[1])/(su1-su0), duz=(PU[2]-O3[2])/(su1-su0);
        double dvx=(PV[0]-O3[0])/(sv1-sv0), dvy=(PV[1]-O3[1])/(sv1-sv0), dvz=(PV[2]-O3[2])/(sv1-sv0);
        double guu=dux*dux+duy*duy+duz*duz, guv=dux*dvx+duy*dvy+duz*dvz, gvv=dvx*dvx+dvy*dvy+dvz*dvz;
        double gdet = guu*gvv - guv*guv;
        double psz = O3.distance(PW);
        bool affine = S.degree(0)==1 && S.degree(1)==1 && gdet > 1e-20
            && std::abs(PW[0]-PU[0]-PV[0]+O3[0]) + std::abs(PW[1]-PU[1]-PV[1]+O3[1])
             + std::abs(PW[2]-PU[2]-PV[2]+O3[2]) < psz*1e-9 + 1e-12;
        if (!affine) continue;
        auto to_uv = [&](const Point& P) -> Point {
            double wx=P[0]-O3[0], wy=P[1]-O3[1], wz=P[2]-O3[2];
            double bu=wx*dux+wy*duy+wz*duz, bv=wx*dvx+wy*dvy+wz*dvz;
            return Point(su0 + (bu*gvv - bv*guv)/gdet, sv0 + (guu*bv - guv*bu)/gdet, 0.0);
        };
        std::vector<Point> uvs;
        uvs.reserve(gpts.size());
        for (const auto& g : gpts) uvs.push_back(to_uv(g));
        // direction: the replacement must traverse like the OLD pcurve so trim.reversed stays valid
        auto pd = pc.domain();
        Point old_s = pc.point_at(pd.first), old_e = pc.point_at(pd.second);
        double d2s = old_s.distance(uvs.front()), d2e = old_s.distance(uvs.back());
        if (d2e < d2s) std::reverse(uvs.begin(), uvs.end());
        // sanity: endpoints must land where the old pcurve's did (sew-tol scale in UV)
        double span_uv = std::max(su1-su0, sv1-sv0);
        if (uvs.front().distance(old_s) > span_uv*2e-2 || uvs.back().distance(old_e) > span_uv*2e-2) continue;
        NurbsCurve npc = NurbsCurve::create(false, 1, uvs);
        if (npc.is_valid()) m_curves_2d[c2] = npc;
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
    auto face_sample = [&](const BRep& X, int fi, double* ou = nullptr, double* ov = nullptr) -> Point {
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
                int n = std::min(std::max(pc.cv_count() * 3, 6), 1024);
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
            if (ou) *ou = 0.5*(u0+u1); if (ov) *ov = 0.5*(v0+v1);
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
        if (ou) *ou = cu; if (ov) *ov = cv;
        return srf.point_at(cu, cv);
    };

    // Classify each imprinted fragment inside/outside the other solid, select per op.
    // Same-domain (ON) pieces -- a fragment lying ON the other solid's boundary, e.g. a
    // coplanar cone base on a box face -- follow OCCT's BOPAlgo_BOP rules: the pair decision
    // is made once on the A side (FUSE/COMMON keep A's copy iff the outward normals agree,
    // CUT keeps A's copy iff they oppose) and the B copy is always dropped. Detection is by
    // probing containment a step along the face normal to BOTH sides: a regular fragment
    // answers the same on both sides, an ON fragment straddles.
    double on_eps;
    {
        double xmn=1e300,ymn=1e300,zmn=1e300,xmx=-1e300,ymx=-1e300,zmx=-1e300;
        for (const auto& q : m_vertices) { xmn=std::min(xmn,q[0]); ymn=std::min(ymn,q[1]); zmn=std::min(zmn,q[2]);
            xmx=std::max(xmx,q[0]); ymx=std::max(ymx,q[1]); zmx=std::max(zmx,q[2]); }
        double dg = std::sqrt((xmx-xmn)*(xmx-xmn)+(ymx-ymn)*(ymx-ymn)+(zmx-zmn)*(zmx-zmn));
        on_eps = (dg > 0 ? dg : 1.0) * 2e-3;
    }
    std::vector<int> keptA, keptB; std::vector<bool> revB;
    auto classify = [&](const BRep& X2, const BRep& solid, const Mesh& solid_mesh,
                        const PrimSolid& prim, const BRep& own, const Mesh& own_mesh,
                        const PrimSolid& own_prim, bool is_first,
                        std::vector<int>& kept, std::vector<bool>* rev) {
        auto in_other = [&](const Point& q) {
            return prim.kind ? inside_prim(prim, q, prim.tol) : solid.contains_point(solid_mesh, q);
        };
        auto in_own = [&](const Point& q) {
            return own_prim.kind ? inside_prim(own_prim, q, own_prim.tol) : own.contains_point(own_mesh, q);
        };
        for (int fi = 0; fi < (int)X2.m_faces.size(); ++fi) {
            if (X2.m_faces[fi].surface_index < 0) continue;
            double cu = 0, cv2 = 0;
            Point sp = face_sample(X2, fi, &cu, &cv2);
            Vector nr = X2.m_surfaces[X2.m_faces[fi].surface_index].normal_at(cu, cv2);
            double nl = nr.magnitude();
            Point pp2(sp[0]+on_eps*nr[0]/(nl>1e-12?nl:1.0), sp[1]+on_eps*nr[1]/(nl>1e-12?nl:1.0), sp[2]+on_eps*nr[2]/(nl>1e-12?nl:1.0));
            Point pm2(sp[0]-on_eps*nr[0]/(nl>1e-12?nl:1.0), sp[1]-on_eps*nr[1]/(nl>1e-12?nl:1.0), sp[2]-on_eps*nr[2]/(nl>1e-12?nl:1.0));
            bool in_p = nl > 1e-12 && in_other(pp2);
            bool in_m = nl > 1e-12 && in_other(pm2);
            bool keep = false, r = false;
            if (nl > 1e-12 && in_p != in_m) {
                // ON piece: orient nr outward via the OWN solid, then compare with the side
                // the other solid occupies.
                bool own_p = in_own(pp2);
                bool same_orient = own_p ? in_p : in_m;   // other occupies this face's inner side
                if (op == BooleanOp::Union)             keep = is_first && same_orient;
                else if (op == BooleanOp::Intersection) keep = is_first && same_orient;
                else                                    keep = is_first && !same_orient;
            } else {
                bool inside = in_other(sp);
                if (op == BooleanOp::Union) keep = !inside;
                else if (op == BooleanOp::Intersection) keep = inside;
                else { if (is_first) keep = !inside; else { keep = inside; r = true; } }
            }
            if (keep) { kept.push_back(fi); if (rev) rev->push_back(r); }
        }
    };
    classify(A2, other, meshB, primB, *this, meshA, primA, true, keptA, nullptr);
    classify(B2, *this, meshA, primA, other, meshB, primB, false, keptB, &revB);   lap("classify");

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
    // SameParameter-lite (planar pcurves rebuilt from the curved side's lifted boundary) is
    // gated OFF: it only helps once ALL section arcs are pullback-quality on both surfaces --
    // with mixed-quality copies (box x tor: e16-type arcs sit 5e-4 off the plane) the rebuilt
    // disks inherit that bias and the volume moves AWAY from truth (+9e-6 vs +1.8e-6).
    if (std::getenv("SESSION_SAMEPARAM")) { result.sameparameter_planar_pcurves(); lap("sameparam"); }
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
