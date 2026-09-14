#include "mini_test.h"
#include "xform.h"
#include "point.h"
#include "vector.h"
#include "mesh.h"
#include "line.h"
#include "plane.h"
#include "polyline.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Xform", "Constructor") {
    Xform x;
    double m00 = x.m[0];
    double m11 = x.m[5];
    double m22 = x.m[10];
    double m33 = x.m[15];
    bool is_id = x.is_identity();
    Xform xfrom = Xform::from_matrix({
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        5.0, 10.0, 15.0, 1.0,
    });
    std::string xstr = x.str();
    std::string xrepr = x.repr();
    Xform xcopy = x;
    Xform xother;
    Xform t = Xform::translation(10.0, 0.0, 0.0);
    Xform s = Xform::scale_xyz(2.0, 1.0, 1.0);
    Xform combined = t * s;
    Point p(1.0, 0.0, 0.0);
    Point result = p.transformed(combined);
    Xform t2 = Xform::translation(10.0, 0.0, 0.0);
    t2 *= s;
    p = Point(1.0, 0.0, 0.0);
    Point result2 = p.transformed(t2);

    MINI_CHECK(x.name == "my_xform");
    MINI_CHECK(!x.guid().empty());
    MINI_CHECK(m00 == 1.0 && m11 == 1.0 && m22 == 1.0 && m33 == 1.0);
    MINI_CHECK(is_id);
    MINI_CHECK(xfrom.m[12] == 5.0 && xfrom.m[13] == 10.0 && xfrom.m[14] == 15.0);
    MINI_CHECK(xstr == "[1.000000, 0.000000, 0.000000, 0.000000]\n[0.000000, 1.000000, 0.000000, 0.000000]\n[0.000000, 0.000000, 1.000000, 0.000000]\n[0.000000, 0.000000, 0.000000, 1.000000]");
    MINI_CHECK(xrepr == "Xform(my_xform, " + x.guid().substr(0, 8) + ")");
    MINI_CHECK(xcopy == x && xcopy.guid() != x.guid());
    MINI_CHECK(xother == x);
    MINI_CHECK(xfrom != x);
    MINI_CHECK(result[0] == 12.0 && result[1] == 0.0 && result[2] == 0.0);
    MINI_CHECK(result2[0] == 12.0 && result2[1] == 0.0 && result2[2] == 0.0);
}

MINI_TEST("Xform", "Translation") {
    Xform xf = Xform::translation(1.5, 1.0, 0.5);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(0.5, 0, -0.5)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(2.5, 0, -0.5)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(2.5, 2, -0.5)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(0.5, 2, -0.5)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(0.5, 0, 1.5)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(2.5, 0, 1.5)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(2.5, 2, 1.5)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(0.5, 2, 1.5)));
}

MINI_TEST("Xform", "Rotation X") {
    double s = std::sqrt(2.0);
    Xform xf = Xform::rotation_x(Tolerance::PI / 4.0);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(-1, 0, -s)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(1, 0, -s)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(1, s, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-1, s, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(-1, -s, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(1, -s, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(1, 0, s)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(-1, 0, s)));
}

MINI_TEST("Xform", "Rotation Y") {
    double s = std::sqrt(2.0);
    Xform xf = Xform::rotation_y(Tolerance::PI / 4.0);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(-s, -1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(0, -1, -s)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(0, 1, -s)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-s, 1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(0, -1, s)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(s, -1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(s, 1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(0, 1, s)));
}

MINI_TEST("Xform", "Rotation Z") {
    double s = std::sqrt(2.0);
    Xform xf = Xform::rotation_z(Tolerance::PI / 4.0);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(0, -s, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(s, 0, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(0, s, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-s, 0, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(0, -s, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(s, 0, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(0, s, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(-s, 0, 1)));
}

MINI_TEST("Xform", "Rotation Axis") {
    Vector axis(1.0, 1.0, 1.0);
    Xform xf = Xform::rotation(axis, 2.0 * Tolerance::PI / 4.0);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    double t = 1.0 / 3.0;
    double k = 2.0 / std::sqrt(3.0);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(-1, -1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(-t, -t+k, -t-k)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(t-k, t+k, t)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-t-k, -t, -t+k)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(-t+k, -t-k, -t)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(t+k, t, t-k)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(1, 1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(t, t-k, t+k)));
}

MINI_TEST("Xform", "Rotation Around Line") {
    double s = std::sqrt(2.0);
    Line line(-1.0, -1.0, -1.0, -1.0, -1.0, 1.0);
    Xform xf = Xform::rotation_around_line(line, Tolerance::PI / 4.0);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(-1, -1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(s-1, s-1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(-1, 2*s-1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-s-1, s-1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(-1, -1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(s-1, s-1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(-1, 2*s-1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(-s-1, s-1, 1)));
}

MINI_TEST("Xform", "Change Basis") {
    Point o0(0, 0, 0);
    Vector x0(1, 0, 0);
    Vector y0(0, 1, 0);
    Vector z0(0, 0, 1);
    Point o1(0.5, -1.0, 0.5);
    Vector x1(1.2, 0.0, 0.0);
    Vector y1(0.3, -1.0, -0.15);
    Vector z1(0.0, 0.0, 1.1);
    Xform xf = Xform::change_basis(o0, x0, y0, z0, o1, x1, y1, z1);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(-1, 0, -0.45)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(1.4, 0, -0.45)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(2, -2, -0.75)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-0.4, -2, -0.75)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(-1, 0, 1.75)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(1.4, 0, 1.75)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(2, -2, 1.45)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(-0.4, -2, 1.45)));
}

MINI_TEST("Xform", "Plane To Plane") {
    Plane pf(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0));
    Plane pt(Point(2, 0, 0), Vector(0, 1, 0), Vector(-1, 0, 0));
    Xform xf = Xform::plane_to_plane(pf, pt);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(1, 1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(1, -1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(3, -1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(3, 1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(1, 1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(1, -1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(3, -1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(3, 1, 1)));
}

MINI_TEST("Xform", "Scale XYZ") {
    Xform xf = Xform::scale_xyz(1.5, 1.2, 1.8);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(-1.5, -1.2, -1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(1.5, -1.2, -1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(1.5, 1.2, -1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-1.5, 1.2, -1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(-1.5, -1.2, 1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(1.5, -1.2, 1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(1.5, 1.2, 1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(-1.5, 1.2, 1.8)));
}

MINI_TEST("Xform", "Scale Uniform") {
    Point c(0, 0, 0);
    Xform xf = Xform::scale_uniform(c, 2.0);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(-2, -2, -2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(2, -2, -2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(2, 2, -2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-2, 2, -2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(-2, -2, 2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(2, -2, 2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(2, 2, 2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(-2, 2, 2)));
}

MINI_TEST("Xform", "Scale Non Uniform") {
    Point c(0, 0, 0);
    Xform xf = Xform::scale_non_uniform(c, 1.5, 1.2, 1.8);
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(-1.5, -1.2, -1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(1.5, -1.2, -1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(1.5, 1.2, -1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-1.5, 1.2, -1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(-1.5, -1.2, 1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(1.5, -1.2, 1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(1.5, 1.2, 1.8)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(-1.5, 1.2, 1.8)));
}

MINI_TEST("Xform", "Look At Right Handed") {
    Point eye(0, 3, 0);
    Point target(0, 0, 0);
    Xform xf = Xform::look_at_right_handed(eye, target, Vector(0, 0, 1));
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(1, -1, -4)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(-1, -1, -4)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(-1, -1, -2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(1, -1, -2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(1, 1, -4)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(-1, 1, -4)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(-1, 1, -2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(1, 1, -2)));
}

MINI_TEST("Xform", "Look To Right Handed") {
    Point eye(0, 3, 0);
    Vector direction(0, -1, 0);
    Xform xf = Xform::look_to_right_handed(eye, direction, Vector(0, 0, 1));
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(1, -1, -4)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(-1, -1, -4)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(-1, -1, -2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(1, -1, -2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(1, 1, -4)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(-1, 1, -4)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(-1, 1, -2)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(1, 1, -2)));
}

MINI_TEST("Xform", "Perspective") {
    Xform view = Xform::translation(0, 0, -2);
    Xform proj = Xform::perspective(Tolerance::PI / 2.0, 1.0, 1.0, 3.0);
    Xform xf = proj * view;
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    double t = 1.0 / 3.0;
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(-t, -t, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(t, -t, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(t, t, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-t, t, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(-1, -1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(1, -1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(1, 1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(-1, 1, 0)));
}

MINI_TEST("Xform", "Orthographic") {
    Xform view = Xform::translation(0, 0, -2);
    Xform proj = Xform::orthographic(-1.0, 1.0, -1.0, 1.0, 1.0, 3.0);
    Xform xf = proj * view;
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh result = mesh.transformed(xf);
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(0).value(), Point(-1, -1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(1).value(), Point(1, -1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(2).value(), Point(1, 1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(3).value(), Point(-1, 1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(4).value(), Point(-1, -1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(5).value(), Point(1, -1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(6).value(), Point(1, 1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(result.vertex_point(7).value(), Point(-1, 1, 0)));
}

MINI_TEST("Xform", "Project To Plane") {
    Plane plane(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0));
    Xform move = Xform::translation(0, 0, 1);
    Xform proj = Xform::project_to_plane(plane);
    Xform xf = proj * move;
    auto tp = [&](double x, double y, double z) { return Point(x,y,z).transformed(xf); };
    Polyline outline({
        tp(-1, -1, -1),
        tp(1, -1, -1),
        tp(1, 1, -1),
        tp(-1, 1, -1),
        tp(-1, -1, -1)
    });
    std::vector<Point> pts = outline.get_points();
    MINI_CHECK(TOLERANCE.is_point_close(pts[0], Point(-1, -1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[1], Point(1, -1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[2], Point(1, 1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[3], Point(-1, 1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[4], Point(-1, -1, 0)));
}

MINI_TEST("Xform", "Project To Plane By Axis") {
    Plane plane(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0));
    Vector direction(1, 0, 1);
    Xform move = Xform::translation(0, 0, 1);
    Xform proj = Xform::project_to_plane_by_axis(plane, direction);
    Xform xf = proj * move;
    auto tp = [&](double x, double y, double z) { return Point(x,y,z).transformed(xf); };
    Polyline outline({
        tp(-1, -1, 1),
        tp(1, -1, -1),
        tp(1, 1, -1),
        tp(-1, 1, 1),
        tp(-1, -1, 1)
    });
    std::vector<Point> pts = outline.get_points();
    MINI_CHECK(TOLERANCE.is_point_close(pts[0], Point(-3, -1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[1], Point(1, -1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[2], Point(1, 1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[3], Point(-3, 1, 0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[4], Point(-3, -1, 0)));
}

MINI_TEST("Xform", "Inverse") {
    Xform t = Xform::translation(1.0, 0.5, 0.5);
    Xform s = Xform::scale_xyz(1.5, 1.2, 1.3);
    Xform composite = t * s;
    Xform inv = composite.inverse().value();
    Mesh mesh = Mesh::create_box(2, 2, 2);
    Mesh forward = mesh.transformed(composite);
    Mesh roundtrip = forward.transformed(inv);
    MINI_CHECK(TOLERANCE.is_point_close(roundtrip.vertex_point(0).value(), Point(-1, -1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(roundtrip.vertex_point(1).value(), Point(1, -1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(roundtrip.vertex_point(2).value(), Point(1, 1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(roundtrip.vertex_point(3).value(), Point(-1, 1, -1)));
    MINI_CHECK(TOLERANCE.is_point_close(roundtrip.vertex_point(4).value(), Point(-1, -1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(roundtrip.vertex_point(5).value(), Point(1, -1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(roundtrip.vertex_point(6).value(), Point(1, 1, 1)));
    MINI_CHECK(TOLERANCE.is_point_close(roundtrip.vertex_point(7).value(), Point(-1, 1, 1)));

    Xform p = Xform::identity();
    p.m[0] = 1.2;
    p.m[5] = 0.8;
    p.m[10] = 1.1;
    p.m[14] = 0.5;
    p.m[11] = -1.0;
    p.m[15] = 0.0;
    Xform pinv = p.inverse().value();
    Xform prod = p * pinv;
    MINI_CHECK(prod.is_identity());
}

MINI_TEST("Xform", "Transform Point") {
    Xform t = Xform::translation(10.0, 20.0, 30.0);
    Xform s = Xform::scale_xyz(2.0, 3.0, 4.0);
    Xform composite = t * s;
    Point p = composite.transform_point(Point(1.0, 1.0, 1.0));
    MINI_CHECK(TOLERANCE.is_point_close(p, Point(12.0, 23.0, 34.0)));

    Xform pr = Xform::identity();
    pr.m[0] = 1.2;
    pr.m[5] = 0.8;
    pr.m[10] = 1.1;
    pr.m[14] = 0.5;
    pr.m[11] = -1.0;
    pr.m[15] = 0.0;
    Point q = pr.transform_point(Point(1.0, 1.0, 2.0));
    MINI_CHECK(TOLERANCE.is_point_close(q, Point(-0.6, -0.4, -1.35)));
}

MINI_TEST("Xform", "Transform Vector") {
    Xform t = Xform::translation(10.0, 20.0, 30.0);
    Xform s = Xform::scale_xyz(2.0, 3.0, 4.0);
    Xform composite = t * s;
    Vector v = composite.transform_vector(Vector(1.0, 1.0, 1.0));
    MINI_CHECK(TOLERANCE.is_vector_close(v, Vector(2.0, 3.0, 4.0)));

    Xform r = Xform::rotation_z(90.0, true);
    Vector u = r.transform_vector(Vector::x_axis());
    MINI_CHECK(TOLERANCE.is_vector_close(u, Vector::y_axis()));
}

MINI_TEST("Xform", "To Cols") {
    Xform xf = Xform::translation(1.0, 2.0, 3.0);
    auto cols = xf.to_cols();
    MINI_CHECK(TOLERANCE.is_close(cols[0][0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(cols[1][1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(cols[2][2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(cols[3][3], 1.0));
    MINI_CHECK(TOLERANCE.is_close(cols[3][0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(cols[3][1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(cols[3][2], 3.0));
}

MINI_TEST("Xform", "Transform Geometry") {
    Xform t = Xform::translation(10.0, 20.0, 30.0);
    Point pt(1.0, 2.0, 3.0);
    Point pt_transformed = pt.transformed(t);
    Vector v(1.0, 0.0, 0.0);
    Vector v_transformed = v.transformed(t);
    Line ln(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    Line ln_transformed = ln.transformed(t);
    Point pl_o(0.0, 0.0, 0.0);
    Vector pl_x(1.0, 0.0, 0.0);
    Vector pl_y(0.0, 1.0, 0.0);
    Plane pl(pl_o, pl_x, pl_y);
    Plane pl_transformed = pl.transformed(t);
    Polyline poly({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)});
    Polyline poly_transformed = poly.transformed(t);
    std::vector<Point> pts = poly_transformed.get_points();

    MINI_CHECK(TOLERANCE.is_point_close(pt_transformed, Point(11.0, 22.0, 33.0)));
    MINI_CHECK(v_transformed[0] == 1.0 && v_transformed[1] == 0.0 && v_transformed[2] == 0.0);
    MINI_CHECK(ln_transformed[0] == 10.0 && ln_transformed[1] == 20.0 && ln_transformed[2] == 30.0);
    MINI_CHECK(ln_transformed[3] == 11.0 && ln_transformed[4] == 20.0 && ln_transformed[5] == 30.0);
    MINI_CHECK(TOLERANCE.is_point_close(pl_transformed.origin(), Point(10.0, 20.0, 30.0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[0], Point(10.0, 20.0, 30.0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[1], Point(11.0, 20.0, 30.0)));
    MINI_CHECK(TOLERANCE.is_point_close(pts[2], Point(11.0, 21.0, 30.0)));
}

MINI_TEST("Xform", "Json Roundtrip") {
    Xform xform = Xform::translation(1.0, 2.0, 3.0);
    xform.name = "test_xform";
    std::string filename = "serialization/test_xform.json";
    xform.file_json_dump(filename);
    Xform loaded = Xform::file_json_load(filename);

    MINI_CHECK(loaded.name == "test_xform");
    MINI_CHECK(loaded.guid() == xform.guid());
    MINI_CHECK(TOLERANCE.is_close(loaded.m[0], 1.0) && TOLERANCE.is_close(loaded.m[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[2], 0.0) && TOLERANCE.is_close(loaded.m[3], 0.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[4], 0.0) && TOLERANCE.is_close(loaded.m[5], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[6], 0.0) && TOLERANCE.is_close(loaded.m[7], 0.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[8], 0.0) && TOLERANCE.is_close(loaded.m[9], 0.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[10], 1.0) && TOLERANCE.is_close(loaded.m[11], 0.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[12], 1.0) && TOLERANCE.is_close(loaded.m[13], 2.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[14], 3.0) && TOLERANCE.is_close(loaded.m[15], 1.0));
}

MINI_TEST("Xform", "Protobuf Roundtrip") {
    Xform xform = Xform::translation(1.0, 2.0, 3.0);
    xform.name = "test_xform_proto";
    std::string filename = "serialization/test_xform.bin";
    const std::string guid = xform.guid();
    xform.pb_dump(filename);
    Xform loaded = Xform::pb_load(filename);

    MINI_CHECK(loaded.name == "test_xform_proto");
    MINI_CHECK(loaded.guid() == guid);
    MINI_CHECK(TOLERANCE.is_close(loaded.m[0], 1.0) && TOLERANCE.is_close(loaded.m[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[2], 0.0) && TOLERANCE.is_close(loaded.m[3], 0.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[4], 0.0) && TOLERANCE.is_close(loaded.m[5], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[6], 0.0) && TOLERANCE.is_close(loaded.m[7], 0.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[8], 0.0) && TOLERANCE.is_close(loaded.m[9], 0.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[10], 1.0) && TOLERANCE.is_close(loaded.m[11], 0.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[12], 1.0) && TOLERANCE.is_close(loaded.m[13], 2.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[14], 3.0) && TOLERANCE.is_close(loaded.m[15], 1.0));
}

MINI_TEST("Xform", "From Change Of Basis") {
    Polyline rect0({
        Point(0.0, 0.0, 0.0),
        Point(2.0, 0.0, 0.0),
        Point(2.0, 3.0, 0.0),
        Point(0.0, 3.0, 0.0),
    });
    Polyline rect1({Point(0.0, 0.0, 4.0)});
    Xform xf = Xform::from_change_of_basis(rect0, rect1);

    MINI_CHECK(TOLERANCE.is_close(xf.m[12], 1.0));
    MINI_CHECK(TOLERANCE.is_close(xf.m[13], 1.5));
    MINI_CHECK(TOLERANCE.is_close(xf.m[14], 2.0));
}

} // namespace session_cpp
