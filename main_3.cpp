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

static void place_line(Session& session, Line ln, double& x, double y, double& my) {
    auto bb = BoundingBox::from_line(ln);
    double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
    ln.xform = Xform::translation(sx, sy, 0);
    ln.transform();
    session.add_line(std::make_shared<Line>(ln));
    auto bb2 = BoundingBox::from_line(ln);
    x = bb2.max_point()[0] + GAP;
    my = std::max(my, bb2.max_point()[1]);
}

static void place_polyline(Session& session, Polyline pl, double& x, double y, double& my) {
    auto bb = BoundingBox::from_polyline(pl);
    double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
    pl.xform = Xform::translation(sx, sy, 0);
    pl.transform();
    session.add_polyline(std::make_shared<Polyline>(pl));
    auto bb2 = BoundingBox::from_polyline(pl);
    x = bb2.max_point()[0] + GAP;
    my = std::max(my, bb2.max_point()[1]);
}

static void place_curve(Session& session, NurbsCurve crv, double& x, double y, double& my) {
    auto bb = BoundingBox::from_nurbscurve(crv);
    double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
    crv.transform(Xform::translation(sx, sy, 0));
    session.add_nurbscurve(std::make_shared<NurbsCurve>(crv));
    auto bb2 = BoundingBox::from_nurbscurve(crv);
    x = bb2.max_point()[0] + GAP;
    my = std::max(my, bb2.max_point()[1]);
}

static void place_surface(Session& session, NurbsSurface srf, double& x, double y, double& my,
                          double mesh_angle = 25, double max_edge = 0.0) {
    auto bb = BoundingBox::from_nurbssurface(srf);
    double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
    srf.transform(Xform::translation(sx, sy, 0));
    session.add_nurbssurface(std::make_shared<NurbsSurface>(srf));
    Mesh m = srf.mesh(mesh_angle, max_edge);
    session.add_mesh(std::make_shared<Mesh>(m));
    auto bb2 = BoundingBox::from_nurbssurface(srf);
    x = bb2.max_point()[0] + GAP;
    my = std::max(my, bb2.max_point()[1]);
}

static void place_mesh(Session& session, Mesh mesh, double& x, double y, double& my) {
    auto bb = BoundingBox::from_mesh(mesh);
    double sx = x - bb.min_point()[0], sy = y - bb.min_point()[1];
    mesh.xform = Xform::translation(sx, sy, 0);
    mesh = mesh.transformed();
    session.add_mesh(std::make_shared<Mesh>(mesh));
    auto bb2 = BoundingBox::from_mesh(mesh);
    x = bb2.max_point()[0] + GAP;
    my = std::max(my, bb2.max_point()[1]);
}

#define NEXT_ROW() do { x = 0; y = my + GAP; my = y; } while(0)

int main() {
    Session session("primitives");
    double x = 0, y = 0, my = 0;

    // Row 0: Lines & Polylines
    auto ln = Line(0, 0, 0, 0, 0, 5); ln.name = "line";
    place_line(session, ln, x, y, my);
    auto pl = Polyline({Point(0,0,0), Point(1,2,0), Point(2,0,0), Point(3,2,0), Point(4,0,0)});
    pl.name = "zigzag";
    place_polyline(session, pl, x, y, my);

    // Row 1: Mesh primitives
    NEXT_ROW();
    auto m_arrow = Primitives::arrow_mesh(Line(0,0,0, 0,0,8), 1.0); m_arrow.name = "arrow";
    place_mesh(session, m_arrow, x, y, my);
    auto m_cyl = Primitives::cylinder_mesh(Line(0,0,0, 0,0,8), 1.0); m_cyl.name = "cylinder";
    place_mesh(session, m_cyl, x, y, my);
    auto m_tet = Primitives::tetrahedron(3.0); m_tet.name = "tetrahedron";
    place_mesh(session, m_tet, x, y, my);
    auto m_cube = Primitives::cube(3.0); m_cube.name = "cube";
    place_mesh(session, m_cube, x, y, my);
    auto m_oct = Primitives::octahedron(3.0); m_oct.name = "octahedron";
    place_mesh(session, m_oct, x, y, my);
    auto m_ico = Primitives::icosahedron(3.0); m_ico.name = "icosahedron";
    place_mesh(session, m_ico, x, y, my);
    auto m_dod = Primitives::dodecahedron(3.0); m_dod.name = "dodecahedron";
    place_mesh(session, m_dod, x, y, my);

    // Row 2: Curve primitives
    NEXT_ROW();
    auto c_polyline = NurbsCurve::create(false, 1, {
        Point(0,0,0), Point(1,2,0), Point(2,0,0), Point(3,2,0), Point(4,0,0)});
    c_polyline.name = "polyline";
    place_curve(session, c_polyline, x, y, my);
    auto c_circle = Primitives::circle(0, 0, 0, 1.0); c_circle.name = "circle";
    place_curve(session, c_circle, x, y, my);
    auto c_ellipse = Primitives::ellipse(0, 0, 0, 2.0, 1.0); c_ellipse.name = "ellipse";
    place_curve(session, c_ellipse, x, y, my);
    auto c_arc = Primitives::arc(Point(0,0,0), Point(1,1,0), Point(2,0,0)); c_arc.name = "arc";
    place_curve(session, c_arc, x, y, my);
    auto c_parabola = Primitives::parabola(Point(-1,1,0), Point(0,0,0), Point(1,1,0)); c_parabola.name = "parabola";
    place_curve(session, c_parabola, x, y, my);
    auto c_hyperbola = Primitives::hyperbola(Point(0,0,0), 1.0, 1.0, 1.0); c_hyperbola.name = "hyperbola";
    place_curve(session, c_hyperbola, x, y, my);
    auto c_spiral = Primitives::spiral(1.0, 2.0, 1.0, 5.0); c_spiral.name = "spiral";
    place_curve(session, c_spiral, x, y, my);

    // Row 3: Surface primitives (each with mesh)
    NEXT_ROW();
    auto s_cyl = Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0); s_cyl.name = "cylinder";
    place_surface(session, s_cyl, x, y, my);
    auto s_cone = Primitives::cone_surface(0, 0, 0, 1.0, 5.0); s_cone.name = "cone";
    place_surface(session, s_cone, x, y, my, 15);

    auto s_sphere = Primitives::sphere_surface(0, 0, 0, 5.0); s_sphere.name = "sphere";
    place_surface(session, s_sphere, x, y, my);

    // quad_sphere — manual placement (excluded from bb rewrite per user request)
    auto quad_faces = Primitives::quad_sphere(0, 0, 0, 5.0);
    {
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
            session.add_nurbssurface(std::make_shared<NurbsSurface>(quad_faces[i]));
            Mesh m = quad_faces[i].mesh(10, 0.5);
            session.add_mesh(std::make_shared<Mesh>(m));
        }
        x += (hi[0] - lo[0]) + GAP;
        my = std::max(my, y + (hi[1] - lo[1]));
    }

    auto s_torus = Primitives::torus_surface(0, 0, 0, 3.0, 1.0); s_torus.name = "torus";
    place_surface(session, s_torus, x, y, my);

    // Row 4: Surface factory methods
    NEXT_ROW();
    auto crvA = NurbsCurve::create(false, 1, {Point(0,0,0), Point(0,5,5)});
    auto crvB = NurbsCurve::create(false, 1, {Point(5,0,5), Point(5,5,0)});
    auto s_ruled = Primitives::create_ruled(crvA, crvB); s_ruled.name = "ruled";
    place_surface(session, s_ruled, x, y, my, 10, 0.3);

    auto c_quad = NurbsCurve::create(false, 1, {
        Point(0,0,0), Point(4,0,0), Point(4,3,0), Point(0,3,0), Point(0,0,0)});
    auto s_planar = Primitives::create_planar(c_quad); s_planar.name = "planar";
    place_surface(session, s_planar, x, y, my);

    auto c_ext = NurbsCurve::create(false, 2, {Point(0,0,0), Point(3,5,0), Point(7,0,0)});
    auto s_extrusion = Primitives::create_extrusion(c_ext, Vector(0,1,5)); s_extrusion.name = "extrusion";
    place_surface(session, s_extrusion, x, y, my);

    auto s_loft = Primitives::create_loft({
        Primitives::circle(0, 0, 0.0, 2.0),
        Primitives::circle(0, 0, 2.0, 1.0),
        Primitives::circle(0, 0, 4.0, 1.5),
        Primitives::circle(0, 0, 6.0, 0.8)}, 3);
    s_loft.name = "loft";
    place_surface(session, s_loft, x, y, my);

    auto c_vase = NurbsCurve::create(false, 3, {
        Point(1.5,0,0), Point(1.5,0,0.3), Point(0.3,0,0.5),
        Point(0.3,0,2.5), Point(0.2,0,3.0), Point(2.0,0,4.5), Point(1.8,0,5.0)});
    auto s_revolve = Primitives::create_revolve(c_vase, Point(0,0,0), Vector(0,0,1)); s_revolve.name = "revolve";
    place_surface(session, s_revolve, x, y, my);

    NurbsCurve rail = NurbsCurve::create(false, 2, {Point(0,0,0), Point(0,5,0), Point(2,9,0)});
    NurbsCurve profile = Primitives::circle(0, 0, 0, 1.0);
    auto s_sweep = Primitives::create_sweep1(rail, profile); s_sweep.name = "sweep";
    place_surface(session, s_sweep, x, y, my);

    NurbsCurve south = NurbsCurve::create(false, 3, {Point(0,0,0), Point(0,2,3), Point(0,5,3), Point(0,7,0)});
    NurbsCurve west  = NurbsCurve::create(false, 2, {Point(5,0,0), Point(2.5,0,3.5), Point(0,0,0)});
    NurbsCurve north = NurbsCurve::create(false, 3, {Point(5,0,0), Point(5,2,3), Point(5,5,3), Point(5,7,0)});
    NurbsCurve east  = NurbsCurve::create(false, 2, {Point(5,7,0), Point(2.5,7,3.5), Point(0,7,0)});
    auto s_edge = Primitives::create_edge(south, west, north, east); s_edge.name = "edge";
    place_surface(session, s_edge, x, y, my);

    auto s_folded = Primitives::create_loft({
        NurbsCurve::create(false, 1, {Point(0,0,0), Point(0.1,0,0.3), Point(0.2,0,0), Point(0.3,0,0.3), Point(0.4,0,0)}),
        NurbsCurve::create(false, 1, {Point(0,0.5,0), Point(0.1,0.5,0.3), Point(0.2,0.5,0), Point(0.3,0.5,0.3), Point(0.4,0.5,0)})}, 1);
    s_folded.transpose();
    s_folded.name = "folded";
    place_surface(session, s_folded, x, y, my, 10, 0.05);

    auto s_wave = Primitives::wave_surface(10.0, 2.0); s_wave.name = "wave";
    place_surface(session, s_wave, x, y, my);

    // Row 5: Surface-to-mesh subdivision
    NEXT_ROW();
    auto s_sub = Primitives::create_ruled(
        NurbsCurve::create(false, 2, {Point(0,0,0), Point(3,0,3), Point(6,0,0)}),
        NurbsCurve::create(false, 2, {Point(0,5,0), Point(3,5,3), Point(6,5,0)}));
    auto m_quad = Primitives::quad_mesh(s_sub, 12, 6); m_quad.name = "quad_mesh";
    place_mesh(session, m_quad, x, y, my);
    auto m_diamond = Primitives::diamond_mesh(s_sub, 12, 6); m_diamond.name = "diamond_mesh";
    place_mesh(session, m_diamond, x, y, my);
    auto m_hex = Primitives::hex_mesh(s_sub, 10, 6, 1.0/4.0); m_hex.name = "hex_mesh";
    place_mesh(session, m_hex, x, y, my);

    // Row 6: Sphere subdivision meshes
    NEXT_ROW();
    auto s_sph = Primitives::sphere_surface(0, 0, 0, 3.0);
    auto m_sph_quad = Primitives::quad_mesh(s_sph, 12, 6); m_sph_quad.name = "sphere_quad";
    place_mesh(session, m_sph_quad, x, y, my);
    auto m_sph_diamond = Primitives::diamond_mesh(s_sph, 12, 6); m_sph_diamond.name = "sphere_diamond";
    place_mesh(session, m_sph_diamond, x, y, my);
    auto m_sph_hex = Primitives::hex_mesh(s_sph, 10, 6, 1.0/4.0); m_sph_hex.name = "sphere_hex";
    place_mesh(session, m_sph_hex, x, y, my);

    // Row 7: Cylinder subdivision meshes
    NEXT_ROW();
    auto s_cyl2 = Primitives::cylinder_surface(0, 0, 0, 3.0, 5.0);
    auto m_cyl_quad = Primitives::quad_mesh(s_cyl2, 12, 6); m_cyl_quad.name = "cyl_quad";
    place_mesh(session, m_cyl_quad, x, y, my);
    auto m_cyl_diamond = Primitives::diamond_mesh(s_cyl2, 12, 6); m_cyl_diamond.name = "cyl_diamond";
    place_mesh(session, m_cyl_diamond, x, y, my);
    auto m_cyl_hex = Primitives::hex_mesh(s_cyl2, 10, 6, 1.0/4.0); m_cyl_hex.name = "cyl_hex";
    place_mesh(session, m_cyl_hex, x, y, my);

    // Row 8: Cone subdivision meshes
    NEXT_ROW();
    auto s_cone2 = Primitives::cone_surface(0, 0, 0, 3.0, 5.0);
    auto m_cone_quad = Primitives::quad_mesh(s_cone2, 12, 6); m_cone_quad.name = "cone_quad";
    place_mesh(session, m_cone_quad, x, y, my);
    auto m_cone_diamond = Primitives::diamond_mesh(s_cone2, 12, 6); m_cone_diamond.name = "cone_diamond";
    place_mesh(session, m_cone_diamond, x, y, my);
    auto m_cone_hex = Primitives::hex_mesh(s_cone2, 10, 6, 1.0/4.0); m_cone_hex.name = "cone_hex";
    place_mesh(session, m_cone_hex, x, y, my);

    // Row 9: Folded plates
    NEXT_ROW();
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

        auto bb2 = BoundingBox::from_mesh(fm);
        x = bb2.max_point()[0] + GAP;
        my = std::max(my, bb2.max_point()[1]);
    }

    // Row 10: Mesh loft
    NEXT_ROW();
    {
        Polyline bot({Point(0,0,0), Point(4,0,0), Point(4,3,0), Point(0,3,0), Point(0,0,0)});
        Polyline top({Point(0,0,5), Point(4,0,5), Point(4,3,5), Point(0,3,5), Point(0,0,5)});
        auto m = Mesh::loft({bot}, {top}); m.name = "loft_box";
        place_mesh(session, m, x, y, my);
    }
    {
        Polyline bot({Point(0,0,0), Point(5,0,0), Point(5,5,0), Point(0,5,0), Point(0,0,0)});
        Polyline top({Point(1,1,4), Point(4,1,4), Point(4,4,4), Point(1,4,4), Point(1,1,4)});
        auto m = Mesh::loft({bot}, {top}); m.name = "loft_tapered";
        place_mesh(session, m, x, y, my);
    }
    {
        Polyline ob({Point(0,0,0), Point(6,0,0), Point(6,6,0), Point(0,6,0), Point(0,0,0)});
        Polyline ot({Point(0,0,5), Point(6,0,5), Point(6,6,5), Point(0,6,5), Point(0,0,5)});
        Polyline ib({Point(1,1,0), Point(5,1,0), Point(5,5,0), Point(1,5,0), Point(1,1,0)});
        Polyline it({Point(1,1,5), Point(5,1,5), Point(5,5,5), Point(1,5,5), Point(1,1,5)});
        auto m = Mesh::loft({ob, ib}, {ot, it}); m.name = "loft_one_hole";
        place_mesh(session, m, x, y, my);
    }
    {
        Polyline ob({Point(0,0,0), Point(12,0,0), Point(12,8,0), Point(0,8,0), Point(0,0,0)});
        Polyline ot({Point(0,0,5), Point(12,0,5), Point(12,8,5), Point(0,8,5), Point(0,0,5)});
        Polyline h1b({Point(1,1,0), Point(3,1,0), Point(3,3,0), Point(1,3,0), Point(1,1,0)});
        Polyline h1t({Point(1,1,5), Point(3,1,5), Point(3,3,5), Point(1,3,5), Point(1,1,5)});
        Polyline h2b({Point(5,1,0), Point(7,1,0), Point(7,3,0), Point(5,3,0), Point(5,1,0)});
        Polyline h2t({Point(5,1,5), Point(7,1,5), Point(7,3,5), Point(5,3,5), Point(5,1,5)});
        Polyline h3b({Point(9,4,0), Point(11,4,0), Point(11,7,0), Point(9,7,0), Point(9,4,0)});
        Polyline h3t({Point(9,4,5), Point(11,4,5), Point(11,7,5), Point(9,7,5), Point(9,4,5)});
        auto m = Mesh::loft({ob, h1b, h2b, h3b}, {ot, h1t, h2t, h3t}); m.name = "loft_multi_hole";
        place_mesh(session, m, x, y, my);
    }

    // Row 11: Cross connectors (hex-shell mesh)
    NEXT_ROW();
    {
        std::vector<std::vector<Point>> polys = {
            {Point(-574.485,-574.300,-182.370), Point(-620.030,-480.441,-136.145), Point(-527.510,-476.289,-53.723), Point(-431.548,-637.846,-118.351)},
            {Point(-545.761,-545.585,-211.069), Point(-589.029,-456.419,-167.156), Point(-501.135,-452.475,-88.854), Point(-409.971,-605.954,-150.251)},
            {Point(-224.289,-691.254,-65.097), Point(-125.302,-566.359,58.481), Point(125.182,-566.359,58.481), Point(212.579,-691.254,-65.097)},
            {Point(-213.075,-656.691,-99.659), Point(-119.037,-538.041,17.739), Point(118.923,-538.041,17.739), Point(201.950,-656.691,-99.659)},
            {Point(456.309,-628.504,-127.534), Point(523.010,-466.370,-45.103), Point(622.146,-472.434,-134.186), Point(574.882,-574.852,-181.916)},
            {Point(433.493,-597.079,-158.975), Point(496.859,-443.052,-80.665), Point(591.038,-448.812,-165.294), Point(546.138,-546.110,-210.638)},
            {Point(-431.100,-638.600,-118.653), Point(-527.510,-476.289,-53.723), Point(-481.982,-362.651,39.785), Point(-234.426,-365.823,142.950), Point(-125.302,-566.359,58.481), Point(-223.832,-690.677,-64.527)},
            {Point(-409.545,-606.670,-150.538), Point(-501.135,-452.475,-88.854), Point(-457.883,-344.518,-0.022), Point(-222.705,-347.532,97.985), Point(-119.037,-538.041,17.739), Point(-212.641,-656.144,-99.118)},
            {Point(212.526,-691.178,-65.022), Point(125.182,-566.359,58.481), Point(238.785,-380.979,136.566), Point(478.593,-374.909,36.915), Point(523.010,-466.370,-45.103), Point(455.692,-630.004,-128.297)},
            {Point(201.900,-656.619,-99.589), Point(118.923,-538.041,17.739), Point(226.846,-361.930,91.920), Point(454.664,-356.164,-2.748), Point(496.859,-443.052,-80.665), Point(432.907,-598.504,-159.699)},
            {Point(-481.982,-362.651,39.785), Point(-527.510,-476.289,-53.723), Point(-620.401,-480.458,-136.475), Point(-698.046,-172.509,-57.884), Point(-595.889,-174.723,29.899)},
            {Point(-457.883,-344.518,-0.022), Point(-501.135,-452.475,-88.854), Point(-589.381,-456.435,-167.469), Point(-663.144,-163.884,-92.807), Point(-566.094,-165.987,-9.413)},
            {Point(-125.302,-566.359,58.481), Point(-234.426,-365.823,142.950), Point(-149.707,-204.103,211.069), Point(150.116,-204.103,211.069), Point(238.785,-380.979,136.566), Point(125.182,-566.359,58.481)},
            {Point(-119.037,-538.041,17.739), Point(-222.705,-347.532,97.985), Point(-142.222,-193.898,162.698), Point(142.610,-193.898,162.698), Point(226.846,-361.930,91.920), Point(118.923,-538.041,17.739)},
            {Point(478.593,-374.909,36.915), Point(596.484,-174.710,29.388), Point(698.046,-172.509,-57.884), Point(622.420,-472.451,-134.432), Point(523.010,-466.370,-45.103)},
            {Point(454.664,-356.164,-2.748), Point(566.659,-165.975,-9.899), Point(663.144,-163.884,-92.807), Point(591.299,-448.828,-165.528), Point(496.859,-443.052,-80.665)},
            {Point(-481.982,-362.651,39.785), Point(-595.889,-174.723,29.899), Point(-515.848,0.000,98.678), Point(-247.814,0.000,211.069), Point(-149.707,-204.103,211.069), Point(-234.426,-365.823,142.950)},
            {Point(-457.883,-344.518,-0.022), Point(-566.094,-165.987,-9.413), Point(-490.055,0.000,55.926), Point(-235.423,0.000,162.698), Point(-142.222,-193.898,162.698), Point(-222.705,-347.532,97.985)},
            {Point(238.785,-380.979,136.566), Point(150.116,-204.103,211.069), Point(252.071,0.000,211.069), Point(512.707,0.000,101.376), Point(596.484,-174.710,29.388), Point(478.593,-374.909,36.915)},
            {Point(226.846,-361.930,91.920), Point(142.610,-193.898,162.698), Point(239.467,0.000,162.698), Point(487.072,0.000,58.490), Point(566.659,-165.975,-9.899), Point(454.664,-356.164,-2.748)},
            {Point(-515.848,0.000,98.678), Point(-595.889,-174.723,29.899), Point(-698.269,-172.505,-58.075), Point(-698.269,172.505,-58.075), Point(-595.889,174.723,29.899)},
            {Point(-490.055,0.000,55.926), Point(-566.094,-165.987,-9.413), Point(-663.356,-163.879,-92.989), Point(-663.356,163.879,-92.989), Point(-566.094,165.987,-9.413)},
            {Point(-149.707,-204.103,211.069), Point(-247.814,0.000,211.069), Point(-149.707,204.103,211.069), Point(150.116,204.103,211.069), Point(252.071,0.000,211.069), Point(150.116,-204.103,211.069)},
            {Point(-142.222,-193.898,162.698), Point(-235.423,0.000,162.698), Point(-142.222,193.898,162.698), Point(142.610,193.898,162.698), Point(239.467,0.000,162.698), Point(142.610,-193.898,162.698)},
            {Point(512.707,0.000,101.376), Point(596.484,174.710,29.388), Point(698.269,172.505,-58.075), Point(698.269,-172.505,-58.075), Point(596.484,-174.710,29.388)},
            {Point(487.072,0.000,58.490), Point(566.659,165.975,-9.899), Point(663.356,163.879,-92.989), Point(663.356,-163.879,-92.989), Point(566.659,-165.975,-9.899)},
            {Point(-515.848,0.000,98.678), Point(-595.889,174.723,29.899), Point(-481.982,362.651,39.785), Point(-234.426,365.823,142.950), Point(-149.707,204.103,211.069), Point(-247.814,0.000,211.069)},
            {Point(-490.055,0.000,55.926), Point(-566.094,165.987,-9.413), Point(-457.883,344.518,-0.022), Point(-222.705,347.532,97.985), Point(-142.222,193.898,162.698), Point(-235.423,0.000,162.698)},
            {Point(252.071,0.000,211.069), Point(150.116,204.103,211.069), Point(238.785,380.979,136.566), Point(478.593,374.909,36.915), Point(596.484,174.710,29.388), Point(512.707,0.000,101.376)},
            {Point(239.467,0.000,162.698), Point(142.610,193.898,162.698), Point(226.846,361.930,91.920), Point(454.664,356.164,-2.748), Point(566.659,165.975,-9.899), Point(487.072,0.000,58.490)},
            {Point(-481.982,362.651,39.785), Point(-595.889,174.723,29.899), Point(-698.046,172.509,-57.884), Point(-622.420,472.451,-134.432), Point(-523.550,466.403,-45.588)},
            {Point(-457.883,344.518,-0.022), Point(-566.094,165.987,-9.413), Point(-663.144,163.884,-92.807), Point(-591.299,448.828,-165.528), Point(-497.372,443.083,-81.126)},
            {Point(-149.707,204.103,211.069), Point(-234.426,365.823,142.950), Point(-125.302,566.359,58.481), Point(125.182,566.359,58.481), Point(238.785,380.979,136.566), Point(150.116,204.103,211.069)},
            {Point(-142.222,193.898,162.698), Point(-222.705,347.532,97.985), Point(-119.037,538.041,17.739), Point(118.923,538.041,17.739), Point(226.846,361.930,91.920), Point(142.610,193.898,162.698)},
            {Point(478.593,374.909,36.915), Point(527.834,476.304,-54.011), Point(620.401,480.458,-136.475), Point(698.046,172.509,-57.884), Point(596.484,174.710,29.388)},
            {Point(454.664,356.164,-2.748), Point(501.442,452.489,-89.128), Point(589.381,456.435,-167.469), Point(663.144,163.884,-92.807), Point(566.659,165.975,-9.899)},
            {Point(-481.982,362.651,39.785), Point(-523.550,466.403,-45.588), Point(-441.383,636.016,-121.338), Point(-223.832,690.677,-64.527), Point(-125.302,566.359,58.481), Point(-234.426,365.823,142.950)},
            {Point(-457.883,344.518,-0.022), Point(-497.372,443.083,-81.126), Point(-419.314,604.215,-153.089), Point(-212.641,656.144,-99.118), Point(-119.037,538.041,17.739), Point(-222.705,347.532,97.985)},
            {Point(238.785,380.979,136.566), Point(125.182,566.359,58.481), Point(212.526,691.178,-65.022), Point(446.073,632.424,-125.794), Point(527.834,476.304,-54.011), Point(478.593,374.909,36.915)},
            {Point(226.846,361.930,91.920), Point(118.923,538.041,17.739), Point(201.900,656.619,-99.589), Point(423.769,600.803,-157.321), Point(501.442,452.489,-89.128), Point(454.664,356.164,-2.748)},
            {Point(-622.146,472.434,-134.186), Point(-574.882,574.852,-181.916), Point(-441.854,635.045,-120.904), Point(-523.550,466.403,-45.588)},
            {Point(-591.038,448.812,-165.294), Point(-546.138,546.110,-210.638), Point(-419.761,603.293,-152.677), Point(-497.372,443.083,-81.126)},
            {Point(-125.302,566.359,58.481), Point(-224.289,691.254,-65.097), Point(212.579,691.254,-65.097), Point(125.182,566.359,58.481)},
            {Point(-119.037,538.041,17.739), Point(-213.075,656.691,-99.659), Point(201.950,656.691,-99.659), Point(118.923,538.041,17.739)},
            {Point(527.834,476.304,-54.011), Point(446.780,631.075,-125.173), Point(574.485,574.300,-182.370), Point(620.030,480.441,-136.145)},
            {Point(501.442,452.489,-89.128), Point(424.440,599.521,-156.732), Point(545.761,545.585,-211.069), Point(589.029,456.419,-167.156)},
        };

        // Use only even-indexed polygons (single layer of the hex-shell)
        std::vector<std::vector<Point>> single_layer;
        for (size_t i = 0; i < polys.size(); i += 2)
            single_layer.push_back(polys[i]);
        Mesh cc_mesh = Mesh::from_polygons(single_layer, 1.0);
        double plate_thick = 2.0;
        double conn_w = 30.0;
        double conn_h = 40.0;
        double conn_t = 1.0;
        CrossConnectors cc(cc_mesh, plate_thick, {0.0}, 2, conn_w, conn_h, conn_t, 0.0);

        double sc = 0.02;
        auto bb = BoundingBox::from_mesh(cc.mesh);
        double sx = x - bb.min_point()[0] * sc, sy = y - bb.min_point()[1] * sc;
        auto xf = Xform::translation(sx, sy, 0) * Xform::scale_xyz(sc, sc, sc);

        Mesh cm = cc.mesh;
        cm.name = "cross_connectors";
        cm.xform = xf;
        cm = cm.transformed();
        session.add_mesh(std::make_shared<Mesh>(cm));

        for (size_t i = 0; i < cc.face_polylines.size(); i++) {
            for (auto& pl : cc.face_polylines[i]) {
                Polyline p = pl;
                p.xform = xf;
                p.transform();
                p.name = "face_plate_" + std::to_string(i);
                p.linecolor = Color(200, 100, 50);
                session.add_polyline(std::make_shared<Polyline>(p));
            }
        }
        for (size_t i = 0; i < cc.edge_polylines.size(); i++) {
            for (auto& pl : cc.edge_polylines[i]) {
                Polyline p = pl;
                p.xform = xf;
                p.transform();
                p.name = "connector_" + std::to_string(i);
                p.linecolor = Color(50, 100, 200);
                session.add_polyline(std::make_shared<Polyline>(p));
            }
        }

        auto bb2 = BoundingBox::from_mesh(cm);
        x = bb2.max_point()[0] + GAP;
        my = std::max(my, bb2.max_point()[1]);
    }

    session.pb_dump("C:/pc/3_code/code_rust/session/session_data/primitives.pb");
    std::cout << "Primitives: " << session.objects.nurbscurves->size() << " curves, "
              << session.objects.nurbssurfaces->size() << " surfaces, "
              << session.objects.meshes->size() << " meshes, "
              << session.objects.lines->size() << " lines, "
              << session.objects.polylines->size() << " polylines" << std::endl;
    return 0;
}
