#include "brep.h"
#include "remesh_nurbssurface_grid.h"
#include "primitives.h"
#include "fmt/core.h"
#include <fstream>
#include <cmath>
#include <map>
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
    guid = ::guid();
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
        Point(-hx, -hy, -hz), Point( hx, -hy, -hz),
        Point( hx,  hy, -hz), Point(-hx,  hy, -hz),
        Point(-hx, -hy,  hz), Point( hx, -hy,  hz),
        Point( hx,  hy,  hz), Point(-hx,  hy,  hz)
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
    auto c2d_bot = NurbsCurve::create(false, 1, {Point(dom_u.first, dom_v.first, 0), Point(dom_u.second, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_bot), ei_bot, li_body, false, BRepTrimType::Mated);
    auto c2d_sr = NurbsCurve::create(false, 1, {Point(dom_u.second, dom_v.first, 0), Point(dom_u.second, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li_body, false, BRepTrimType::Seam);
    auto c2d_top = NurbsCurve::create(false, 1, {Point(dom_u.second, dom_v.second, 0), Point(dom_u.first, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_top), ei_top, li_body, true, BRepTrimType::Mated);
    auto c2d_sl = NurbsCurve::create(false, 1, {Point(dom_u.first, dom_v.second, 0), Point(dom_u.first, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_sl), ei_seam, li_body, true, BRepTrimType::Seam);

    // Circular 2D trim in UV space: circle at (0.5,0.5) radius 0.5
    const double cw = std::sqrt(2.0) / 2.0;
    double ccx[] = {1,1,0,-1,-1,-1,0,1,1}, ccy[] = {0,1,1,1,0,-1,-1,-1,0};
    double cwt[] = {1,cw,1,cw,1,cw,1,cw,1}, ckn[] = {0,0,1,1,2,2,3,3,4,4};
    auto make_cap_circle = [&]() {
        NurbsCurve c(3, true, 3, 9);
        for (int i = 0; i < 10; i++) c.set_knot(i, ckn[i]);
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

    auto c2d_south = NurbsCurve::create(false, 1, {Point(dom_u.first, dom_v.first, 0), Point(dom_u.second, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_south), -1, li, false, BRepTrimType::Singular);
    auto c2d_sr = NurbsCurve::create(false, 1, {Point(dom_u.second, dom_v.first, 0), Point(dom_u.second, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li, false, BRepTrimType::Seam);
    auto c2d_north = NurbsCurve::create(false, 1, {Point(dom_u.second, dom_v.second, 0), Point(dom_u.first, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_north), -1, li, false, BRepTrimType::Singular);
    auto c2d_sl = NurbsCurve::create(false, 1, {Point(dom_u.first, dom_v.second, 0), Point(dom_u.first, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_sl), ei_seam, li, true, BRepTrimType::Seam);

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
        Point(-hx, -hy, -hz), Point( hx, -hy, -hz),
        Point( hx,  hy, -hz), Point(-hx,  hy, -hz),
        Point(-hx, -hy,  hz), Point( hx, -hy,  hz),
        Point( hx,  hy,  hz), Point(-hx,  hy,  hz)
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
    NurbsCurve seam_line = NurbsCurve::create(false, 1, {Point(hole_radius, 0, -hz), Point(hole_radius, 0, hz)});
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
    auto c2d_bot = NurbsCurve::create(false, 1, {Point(dom_u.first, dom_v.first, 0), Point(dom_u.second, dom_v.first, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_bot), ei_bot, li_cyl, false, BRepTrimType::Mated);
    auto c2d_sr = NurbsCurve::create(false, 1, {Point(dom_u.second, dom_v.first, 0), Point(dom_u.second, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_sr), ei_seam, li_cyl, false, BRepTrimType::Seam);
    auto c2d_top = NurbsCurve::create(false, 1, {Point(dom_u.second, dom_v.second, 0), Point(dom_u.first, dom_v.second, 0)});
    brep.add_trim(brep.add_curve_2d(c2d_top), ei_top, li_cyl, true, BRepTrimType::Mated);
    auto c2d_sl = NurbsCurve::create(false, 1, {Point(dom_u.first, dom_v.second, 0), Point(dom_u.first, dom_v.first, 0)});
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
            auto tc = NurbsCurve::create(false, 1, {Point(u0, v0, 0), Point(u1, v1, 0)});
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
        for (int i = 0; i < 10; ++i) hole_crv.set_knot(i, ckn[i]);
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
            auto tc = NurbsCurve::create(false, 1, {Point(u0, v0, 0), Point(u1, v1, 0)});
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
        for (int i = 0; i < crv.knot_count(); ++i) crv2d.set_knot(i, crv.knot(i));
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
    for (const auto& e : m_topology_edges) {
        if ((int)e.trim_indices.size() != 2) return false;
    }
    return true;
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

        TrimmedSurface ts;
        ts.m_surface = srf;
        for (int li : face.loop_indices) {
            if (li < 0 || li >= (int)m_loops.size()) continue;
            const auto& loop = m_loops[li];
            std::vector<Point> loop_pts;
            for (int ti : loop.trim_indices) {
                if (ti < 0 || ti >= (int)m_trims.size()) continue;
                const auto& trim = m_trims[ti];
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
        if (face.reversed) {
            for (auto& [vk, vd] : fm.vertex) {
                auto n = vd.normal();
                if (n) vd.set_normal(-(*n)[0], -(*n)[1], -(*n)[2]);
            }
        }
        for (auto& [fk, fverts] : fm.face) {
            std::vector<Point> poly;
            for (auto vi : fverts)
                poly.push_back(fm.vertex.at(vi).position());
            all_polygons.push_back(poly);
        }
    }
    return Mesh::from_polylines(all_polygons, 1e-6);
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
    return m_surfaces[si].normal_at(u, v);
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
    j["guid"] = guid;
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
    if (data.contains("guid")) b.guid = data["guid"];
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

std::string BRep::json_dumps() const { return jsondump().dump(); }
BRep BRep::json_loads(const std::string& json_string) {
    return jsonload(nlohmann::ordered_json::parse(json_string));
}

void BRep::json_dump(const std::string& filename) const {
    std::ofstream file(filename);
    file << jsondump().dump(4);
}

BRep BRep::json_load(const std::string& filename) {
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
    proto.set_guid(guid);
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
    xform_proto->set_guid(xform.guid);
    xform_proto->set_name(xform.name);
    for (int i = 0; i < 16; ++i) xform_proto->add_matrix(xform.m[i]);

    return proto.SerializeAsString();
}

BRep BRep::pb_loads(const std::string& data) {
    session_proto::BRep proto;
    proto.ParseFromString(data);
    BRep b;
    b.guid = proto.guid();
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
    b.xform.guid = xp.guid();
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
