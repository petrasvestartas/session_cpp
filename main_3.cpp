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

static void place_line(Session& session, Line ln, double& x) {
    auto bb = BoundingBox::from_line(ln);
    double shift = x - bb.min_point()[0];
    ln.xform = Xform::translation(shift, 0, 0);
    ln.transform();
    session.add_line(std::make_shared<Line>(ln));
    auto bb2 = BoundingBox::from_line(ln);
    x = bb2.max_point()[0] + GAP;
}

static void place_polyline(Session& session, Polyline pl, double& x) {
    auto bb = BoundingBox::from_polyline(pl);
    double shift = x - bb.min_point()[0];
    pl.xform = Xform::translation(shift, 0, 0);
    pl.transform();
    session.add_polyline(std::make_shared<Polyline>(pl));
    auto bb2 = BoundingBox::from_polyline(pl);
    x = bb2.max_point()[0] + GAP;
}

static void place_curve(Session& session, NurbsCurve crv, double& x) {
    auto bb = BoundingBox::from_nurbscurve(crv);
    double shift = x - bb.min_point()[0];
    crv.transform(Xform::translation(shift, 0, 0));
    session.add_curve(std::make_shared<NurbsCurve>(crv));
    auto bb2 = BoundingBox::from_nurbscurve(crv);
    x = bb2.max_point()[0] + GAP;
}

static void place_surface(Session& session, NurbsSurface srf, double& x,
                          double mesh_angle = 25, double max_edge = 0.0) {
    auto bb = BoundingBox::from_nurbssurface(srf);
    double shift = x - bb.min_point()[0];
    srf.transform(Xform::translation(shift, 0, 0));
    session.add_surface(std::make_shared<NurbsSurface>(srf));
    Mesh m = srf.mesh(mesh_angle, max_edge);
    session.add_mesh(std::make_shared<Mesh>(m));
    auto bb2 = BoundingBox::from_nurbssurface(srf);
    x = bb2.max_point()[0] + GAP;
}

static void place_mesh(Session& session, Mesh mesh, double& x) {
    auto bb = BoundingBox::from_mesh(mesh);
    double shift = x - bb.min_point()[0];
    mesh.xform = Xform::translation(shift, 0, 0);
    mesh = mesh.transformed();
    session.add_mesh(std::make_shared<Mesh>(mesh));
    auto bb2 = BoundingBox::from_mesh(mesh);
    x = bb2.max_point()[0] + GAP;
}

int main() {
    Session session("primitives");
    double x = 0;

    // Line
    place_line(session, Line(0, 0, 0, 0, 0, 5), x);

    // Polyline (zigzag)
    place_polyline(session, Polyline({
        Point(0,0,0), Point(1,2,0), Point(2,0,0), Point(3,2,0), Point(4,0,0)}), x);

    // Mesh primitives
    place_mesh(session, Primitives::arrow_mesh(Line(0,0,0, 0,0,8), 1.0), x);
    place_mesh(session, Primitives::cylinder_mesh(Line(0,0,0, 0,0,8), 1.0), x);
    place_mesh(session, Primitives::tetrahedron(3.0), x);
    place_mesh(session, Primitives::cube(3.0), x);
    place_mesh(session, Primitives::octahedron(3.0), x);
    place_mesh(session, Primitives::icosahedron(3.0), x);
    place_mesh(session, Primitives::dodecahedron(3.0), x);

    // Curve primitives
    place_curve(session, NurbsCurve::create(false, 1, {
        Point(0,0,0), Point(1,2,0), Point(2,0,0), Point(3,2,0), Point(4,0,0)}), x);
    place_curve(session, Primitives::circle(0, 0, 0, 1.0), x);
    place_curve(session, Primitives::ellipse(0, 0, 0, 2.0, 1.0), x);
    place_curve(session, Primitives::arc(Point(0,0,0), Point(1,1,0), Point(2,0,0)), x);
    place_curve(session, Primitives::parabola(Point(-1,1,0), Point(0,0,0), Point(1,1,0)), x);
    place_curve(session, Primitives::hyperbola(Point(0,0,0), 1.0, 1.0, 1.0), x);
    place_curve(session, Primitives::spiral(1.0, 2.0, 1.0, 5.0), x);

    // Surface primitives (each with mesh)
    place_surface(session, Primitives::cylinder_surface(0, 0, 0, 1.0, 5.0), x);
    place_surface(session, Primitives::cone_surface(0, 0, 0, 1.0, 5.0), x, 15);

    place_surface(session, Primitives::torus_surface(0, 0, 0, 3.0, 1.0), x);

    auto crvA = NurbsCurve::create(false, 1, {Point(0,0,0), Point(0,5,5)});
    auto crvB = NurbsCurve::create(false, 1, {Point(5,0,5), Point(5,5,0)});
    place_surface(session, Primitives::create_ruled(crvA, crvB), x, 10, 0.3);

    auto c_quad = NurbsCurve::create(false, 1, {
        Point(0,0,0), Point(4,0,0), Point(4,3,0), Point(0,3,0), Point(0,0,0)});
    place_surface(session, Primitives::create_planar(c_quad), x);

    auto c_ext = NurbsCurve::create(false, 2, {Point(0,0,0), Point(3,5,0), Point(7,0,0)});
    place_surface(session, Primitives::create_extrusion(c_ext, Vector(0,1,5)), x);

    place_surface(session, Primitives::create_loft({
        Primitives::circle(0, 0, 0.0, 2.0),
        Primitives::circle(0, 0, 2.0, 1.0),
        Primitives::circle(0, 0, 4.0, 1.5),
        Primitives::circle(0, 0, 6.0, 0.8)}, 3), x);

    auto c_vase = NurbsCurve::create(false, 3, {
        Point(1.5,0,0), Point(1.5,0,0.3), Point(0.3,0,0.5),
        Point(0.3,0,2.5), Point(0.2,0,3.0), Point(2.0,0,4.5), Point(1.8,0,5.0)});
    place_surface(session, Primitives::create_revolve(c_vase, Point(0,0,0), Vector(0,0,1)), x);

    NurbsCurve rail = NurbsCurve::create(false, 2, {Point(0,0,0), Point(0,5,0), Point(2,9,0)});
    NurbsCurve profile = Primitives::circle(0, 0, 0, 1.0);
    place_surface(session, Primitives::create_sweep1(rail, profile), x);

    NurbsCurve south = NurbsCurve::create(false, 3, {Point(0,0,0), Point(0,2,3), Point(0,5,3), Point(0,7,0)});
    NurbsCurve west  = NurbsCurve::create(false, 2, {Point(5,0,0), Point(2.5,0,3.5), Point(0,0,0)});
    NurbsCurve north = NurbsCurve::create(false, 3, {Point(5,0,0), Point(5,2,3), Point(5,5,3), Point(5,7,0)});
    NurbsCurve east  = NurbsCurve::create(false, 2, {Point(5,7,0), Point(2.5,7,3.5), Point(0,7,0)});
    place_surface(session, Primitives::create_edge(south, west, north, east), x);

    session.pb_dump("C:/pc/3_code/code_rust/session/session_data/primitives.pb");
    std::cout << "Primitives: " << session.objects.nurbscurves->size() << " curves, "
              << session.objects.nurbssurfaces->size() << " surfaces, "
              << session.objects.meshes->size() << " meshes, "
              << session.objects.lines->size() << " lines, "
              << session.objects.polylines->size() << " polylines"
              << " | x_extent: " << x - GAP << std::endl;
    return 0;
}
