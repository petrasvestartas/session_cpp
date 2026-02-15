#include "session.h"
#include "nurbssurface.h"
#include "nurbscurve.h"
#include "primitives.h"
#include "mesh.h"
#include "line.h"
#include "polyline.h"
#include "point.h"
#include "vector.h"
#include "boundingbox.h"
#include "xform.h"
#include <iostream>

using namespace session_cpp;

static const double GAP = 2.0;
static const double ROW_GAP = 15.0;

static void place_line(Session& session, Line ln, double& x, double y) {
    auto bb = BoundingBox::from_line(ln);
    double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
    ln.xform = Xform::translation(sx, sy, 0);
    ln.transform();
    session.add_line(std::make_shared<Line>(ln));
    x = BoundingBox::from_line(ln).max_point()[0] + GAP;
}

static void place_polyline(Session& session, Polyline pl, double& x, double y) {
    auto bb = BoundingBox::from_polyline(pl);
    double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
    pl.xform = Xform::translation(sx, sy, 0);
    pl.transform();
    session.add_polyline(std::make_shared<Polyline>(pl));
    x = BoundingBox::from_polyline(pl).max_point()[0] + GAP;
}

static void place_curve(Session& session, NurbsCurve crv, double& x, double y) {
    auto bb = BoundingBox::from_nurbscurve(crv);
    double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
    crv.transform(Xform::translation(sx, sy, 0));
    session.add_curve(std::make_shared<NurbsCurve>(crv));
    x = BoundingBox::from_nurbscurve(crv).max_point()[0] + GAP;
}

static void place_surface(Session& session, NurbsSurface srf, double& x, double y,
                          double mesh_angle = 25, double max_edge = 0.0) {
    auto bb = BoundingBox::from_nurbssurface(srf);
    double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
    srf.transform(Xform::translation(sx, sy, 0));
    session.add_surface(std::make_shared<NurbsSurface>(srf));
    Mesh m = srf.mesh(mesh_angle, max_edge);
    session.add_mesh(std::make_shared<Mesh>(m));
    x = BoundingBox::from_nurbssurface(srf).max_point()[0] + GAP;
}

static void place_mesh(Session& session, Mesh mesh, double& x, double y) {
    auto bb = BoundingBox::from_mesh(mesh);
    double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
    mesh.xform = Xform::translation(sx, sy, 0);
    mesh = mesh.transformed();
    session.add_mesh(std::make_shared<Mesh>(mesh));
    x = BoundingBox::from_mesh(mesh).max_point()[0] + GAP;
}

int main() {
    Session session("primitives");
    double x = 0, y = 0;

    // Row 0: Lines & Polylines
    auto ln = Line(0, 0, 0, 0, 0, 5); ln.name = "line";
    place_line(session, ln, x, y);
    auto pl = Polyline({Point(0,0,0), Point(1,2,0), Point(2,0,0), Point(3,2,0), Point(4,0,0)});
    pl.name = "zigzag";
    place_polyline(session, pl, x, y);

    // Row 1: Mesh primitives
    x = 0; y += ROW_GAP;
    auto m_arrow = Primitives::arrow_mesh(Line(0,0,0, 0,0,8), 1.0); m_arrow.name = "arrow";
    place_mesh(session, m_arrow, x, y);
    auto m_cyl = Primitives::cylinder_mesh(Line(0,0,0, 0,0,8), 1.0); m_cyl.name = "cylinder";
    place_mesh(session, m_cyl, x, y);
    auto m_tet = Primitives::tetrahedron(3.0); m_tet.name = "tetrahedron";
    place_mesh(session, m_tet, x, y);
    auto m_cube = Primitives::cube(3.0); m_cube.name = "cube";
    place_mesh(session, m_cube, x, y);
    auto m_oct = Primitives::octahedron(3.0); m_oct.name = "octahedron";
    place_mesh(session, m_oct, x, y);
    auto m_ico = Primitives::icosahedron(3.0); m_ico.name = "icosahedron";
    place_mesh(session, m_ico, x, y);
    auto m_dod = Primitives::dodecahedron(3.0); m_dod.name = "dodecahedron";
    place_mesh(session, m_dod, x, y);

    // Row 2: Curve primitives
    x = 0; y += ROW_GAP;
    auto c_polyline = NurbsCurve::create(false, 1, {
        Point(0,0,0), Point(1,2,0), Point(2,0,0), Point(3,2,0), Point(4,0,0)});
    c_polyline.name = "polyline";
    place_curve(session, c_polyline, x, y);
    auto c_circle = Primitives::circle(0, 0, 0, 1.0); c_circle.name = "circle";
    place_curve(session, c_circle, x, y);
    auto c_ellipse = Primitives::ellipse(0, 0, 0, 2.0, 1.0); c_ellipse.name = "ellipse";
    place_curve(session, c_ellipse, x, y);
    auto c_arc = Primitives::arc(Point(0,0,0), Point(1,1,0), Point(2,0,0)); c_arc.name = "arc";
    place_curve(session, c_arc, x, y);
    auto c_parabola = Primitives::parabola(Point(-1,1,0), Point(0,0,0), Point(1,1,0)); c_parabola.name = "parabola";
    place_curve(session, c_parabola, x, y);
    auto c_hyperbola = Primitives::hyperbola(Point(0,0,0), 1.0, 1.0, 1.0); c_hyperbola.name = "hyperbola";
    place_curve(session, c_hyperbola, x, y);
    auto c_spiral = Primitives::spiral(1.0, 2.0, 1.0, 5.0); c_spiral.name = "spiral";
    place_curve(session, c_spiral, x, y);

    // Row 3: Surface primitives (each with mesh)
    x = 0; y += ROW_GAP;
    auto s_cyl = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0); s_cyl.name = "cylinder";
    place_surface(session, s_cyl, x, y);
    auto s_cone = Primitives::cone_surface(0, 0, 0, 1.0, 5.0); s_cone.name = "cone";
    place_surface(session, s_cone, x, y, 15);

    auto s_sphere = Primitives::sphere_surface(0, 0, 0, 5.0); s_sphere.name = "sphere";
    place_surface(session, s_sphere, x, y);

    auto quad_faces = Primitives::quad_sphere(0, 0, 0, 5.0);
    {
        // Sample actual surface points to get true bounding box
        Point lo(1e18,1e18,1e18), hi(-1e18,-1e18,-1e18);
        for (auto& f : quad_faces) {
            for (int i = 0; i <= 4; i++) for (int j = 0; j <= 4; j++) {
                Point p = f.point_at(i/4.0, j/4.0);
                lo = Point(std::min(lo[0],p[0]), std::min(lo[1],p[1]), std::min(lo[2],p[2]));
                hi = Point(std::max(hi[0],p[0]), std::max(hi[1],p[1]), std::max(hi[2],p[2]));
            }
        }
        auto xf = Xform::translation(x - lo[0], y - lo[1], 0);
        for (size_t i = 0; i < quad_faces.size(); i++) {
            quad_faces[i].name = "quad_sphere_" + std::to_string(i);
            quad_faces[i].transform(xf);
            session.add_surface(std::make_shared<NurbsSurface>(quad_faces[i]));
            Mesh m = quad_faces[i].mesh(10, 0.5);
            session.add_mesh(std::make_shared<Mesh>(m));
        }
        x += (hi[0] - lo[0]) + GAP;
    }

    auto s_torus = Primitives::torus_surface(0, 0, 0, 3.0, 1.0); s_torus.name = "torus";
    place_surface(session, s_torus, x, y);

    // Row 4: Surface factory methods
    x = 0; y += ROW_GAP;
    auto crvA = NurbsCurve::create(false, 1, {Point(0,0,0), Point(0,5,5)});
    auto crvB = NurbsCurve::create(false, 1, {Point(5,0,5), Point(5,5,0)});
    auto s_ruled = Primitives::create_ruled(crvA, crvB); s_ruled.name = "ruled";
    place_surface(session, s_ruled, x, y, 10, 0.3);

    auto c_quad = NurbsCurve::create(false, 1, {
        Point(0,0,0), Point(4,0,0), Point(4,3,0), Point(0,3,0), Point(0,0,0)});
    auto s_planar = Primitives::create_planar(c_quad); s_planar.name = "planar";
    place_surface(session, s_planar, x, y);

    auto c_ext = NurbsCurve::create(false, 2, {Point(0,0,0), Point(3,5,0), Point(7,0,0)});
    auto s_extrusion = Primitives::create_extrusion(c_ext, Vector(0,1,5)); s_extrusion.name = "extrusion";
    place_surface(session, s_extrusion, x, y);

    auto s_loft = Primitives::create_loft({
        Primitives::circle(0, 0, 0.0, 2.0),
        Primitives::circle(0, 0, 2.0, 1.0),
        Primitives::circle(0, 0, 4.0, 1.5),
        Primitives::circle(0, 0, 6.0, 0.8)}, 3);
    s_loft.name = "loft";
    place_surface(session, s_loft, x, y);

    auto c_vase = NurbsCurve::create(false, 3, {
        Point(1.5,0,0), Point(1.5,0,0.3), Point(0.3,0,0.5),
        Point(0.3,0,2.5), Point(0.2,0,3.0), Point(2.0,0,4.5), Point(1.8,0,5.0)});
    auto s_revolve = Primitives::create_revolve(c_vase, Point(0,0,0), Vector(0,0,1)); s_revolve.name = "revolve";
    place_surface(session, s_revolve, x, y);

    NurbsCurve rail = NurbsCurve::create(false, 2, {Point(0,0,0), Point(0,5,0), Point(2,9,0)});
    NurbsCurve profile = Primitives::circle(0, 0, 0, 1.0);
    auto s_sweep = Primitives::create_sweep1(rail, profile); s_sweep.name = "sweep";
    place_surface(session, s_sweep, x, y);

    NurbsCurve south = NurbsCurve::create(false, 3, {Point(0,0,0), Point(0,2,3), Point(0,5,3), Point(0,7,0)});
    NurbsCurve west  = NurbsCurve::create(false, 2, {Point(5,0,0), Point(2.5,0,3.5), Point(0,0,0)});
    NurbsCurve north = NurbsCurve::create(false, 3, {Point(5,0,0), Point(5,2,3), Point(5,5,3), Point(5,7,0)});
    NurbsCurve east  = NurbsCurve::create(false, 2, {Point(5,7,0), Point(2.5,7,3.5), Point(0,7,0)});
    auto s_edge = Primitives::create_edge(south, west, north, east); s_edge.name = "edge";
    place_surface(session, s_edge, x, y);

    auto s_folded = Primitives::create_loft({
        NurbsCurve::create(false, 1, {Point(0,0,0), Point(0.1,0,0.3), Point(0.2,0,0), Point(0.3,0,0.3), Point(0.4,0,0)}),
        NurbsCurve::create(false, 1, {Point(0,0.5,0), Point(0.1,0.5,0.3), Point(0.2,0.5,0), Point(0.3,0.5,0.3), Point(0.4,0.5,0)})}, 1);
    s_folded.transpose();
    s_folded.name = "folded";
    place_surface(session, s_folded, x, y, 10, 0.05);

    auto s_wave = Primitives::wave_surface(10.0, 2.0); s_wave.name = "wave";
    place_surface(session, s_wave, x, y);

    // Row 5: Surface-to-mesh subdivision — one simple surface per pattern
    x = 0; y += ROW_GAP;
    auto s_sub = Primitives::create_ruled(
        NurbsCurve::create(false, 2, {Point(0,0,0), Point(3,0,3), Point(6,0,0)}),
        NurbsCurve::create(false, 2, {Point(0,5,0), Point(3,5,3), Point(6,5,0)}));
    auto m_quad = Primitives::quad_mesh(s_sub, 12, 6); m_quad.name = "quad_mesh";
    place_mesh(session, m_quad, x, y);
    auto m_diamond = Primitives::diamond_mesh(s_sub, 12, 6); m_diamond.name = "diamond_mesh";
    place_mesh(session, m_diamond, x, y);
    auto m_hex = Primitives::hex_mesh(s_sub, 10, 6, 1.0/4.0); m_hex.name = "hex_mesh";
    place_mesh(session, m_hex, x, y);

    // Row 6: Sphere subdivision meshes (closed + singular)
    x = 0; y += ROW_GAP;
    auto s_sph = Primitives::sphere_surface(0, 0, 0, 3.0);
    auto m_sph_quad = Primitives::quad_mesh(s_sph, 12, 6); m_sph_quad.name = "sphere_quad";
    place_mesh(session, m_sph_quad, x, y);
    auto m_sph_diamond = Primitives::diamond_mesh(s_sph, 12, 6); m_sph_diamond.name = "sphere_diamond";
    place_mesh(session, m_sph_diamond, x, y);
    auto m_sph_hex = Primitives::hex_mesh(s_sph, 10, 6, 1.0/4.0); m_sph_hex.name = "sphere_hex";
    place_mesh(session, m_sph_hex, x, y);

    // Row 7: Cylinder subdivision meshes (closed, no poles)
    x = 0; y += ROW_GAP;
    auto s_cyl2 = Primitives::cylinder_surface(0, 0, 0, 3.0, 5.0);
    auto m_cyl_quad = Primitives::quad_mesh(s_cyl2, 12, 6); m_cyl_quad.name = "cyl_quad";
    place_mesh(session, m_cyl_quad, x, y);
    auto m_cyl_diamond = Primitives::diamond_mesh(s_cyl2, 12, 6); m_cyl_diamond.name = "cyl_diamond";
    place_mesh(session, m_cyl_diamond, x, y);
    auto m_cyl_hex = Primitives::hex_mesh(s_cyl2, 10, 6, 1.0/4.0); m_cyl_hex.name = "cyl_hex";
    place_mesh(session, m_cyl_hex, x, y);

    // Row 8: Cone subdivision meshes (closed + north pole)
    x = 0; y += ROW_GAP;
    auto s_cone2 = Primitives::cone_surface(0, 0, 0, 3.0, 5.0);
    auto m_cone_quad = Primitives::quad_mesh(s_cone2, 12, 6); m_cone_quad.name = "cone_quad";
    place_mesh(session, m_cone_quad, x, y);
    auto m_cone_diamond = Primitives::diamond_mesh(s_cone2, 12, 6); m_cone_diamond.name = "cone_diamond";
    place_mesh(session, m_cone_diamond, x, y);
    auto m_cone_hex = Primitives::hex_mesh(s_cone2, 10, 6, 1.0/4.0); m_cone_hex.name = "cone_hex";
    place_mesh(session, m_cone_hex, x, y);

    // Row 9: Folded plates (diamond subdivision of curved surface)
    x = 0; y += ROW_GAP;
    {
        auto fp_arc = Primitives::arc(Point(-10, 0, 0), Point(0, 0, 10), Point(10, 0, 0));
        auto fp_srf = Primitives::create_extrusion(fp_arc, Vector(0, 30, 0));
        fp_srf.transpose();
        FoldedPlates fp(fp_srf, 5, 2, 0.5, 0.3);

        auto bb = BoundingBox::from_mesh(fp.mesh);
        double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
        auto xf = Xform::translation(sx, sy, 0);

        Mesh fm = fp.mesh;
        fm.name = "folded_plates";
        fm.xform = xf;
        fm = fm.transformed();
        session.add_mesh(std::make_shared<Mesh>(fm));

        for (size_t i = 0; i < fp.polylines.size(); i++) {
            for (auto& pl : fp.polylines[i]) {
                Polyline p = pl;
                p.xform = xf;
                p.transform();
                p.name = "plate_" + std::to_string(i);
                session.add_polyline(std::make_shared<Polyline>(p));
            }
        }
        for (auto& il : fp.insertion_lines) {
            Line l = il;
            l.xform = xf;
            l.transform();
            l.name = "insertion";
            session.add_line(std::make_shared<Line>(l));
        }

        x = BoundingBox::from_mesh(fm).max_point()[0] + GAP;
    }

    session.pb_dump("C:/pc/3_code/code_rust/session/session_data/primitives.pb");
    std::cout << "Primitives: " << session.objects.nurbscurves->size() << " curves, "
              << session.objects.nurbssurfaces->size() << " surfaces, "
              << session.objects.meshes->size() << " meshes, "
              << session.objects.lines->size() << " lines, "
              << session.objects.polylines->size() << " polylines" << std::endl;
    return 0;
}
