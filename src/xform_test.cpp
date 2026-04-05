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
    // uncomment #include "xform.h"
    // uncomment #include "point.h"

    // Constructor (identity by default)
    Xform x;

    // Matrix access
    double m00 = x.m[0];
    double m11 = x.m[5];
    double m22 = x.m[10];
    double m33 = x.m[15];

    // Check identity
    bool is_id = x.is_identity();

    // From matrix constructor
    Xform xfrom = Xform::from_matrix({
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        5.0, 10.0, 15.0, 1.0,
    });

    // Minimal and Full String Representation
    std::string xstr = x.str();
    std::string xrepr = x.repr();

    // Copy (duplicates everything except guid())
    Xform xcopy = x;
    Xform xother;

    // Matrix multiplication (*)
    Xform t = Xform::translation(10.0, 0.0, 0.0);
    Xform s = Xform::scale_xyz(2.0, 1.0, 1.0);
    Xform combined = t * s;
    Point p(1.0, 0.0, 0.0);
    p.xform = combined;
    Point result = p.transformed();

    // In-place multiplication (*=)
    Xform t2 = Xform::translation(10.0, 0.0, 0.0);
    t2 *= s;
    p = Point(1.0, 0.0, 0.0);
    p.xform = t2;
    Point result2 = p.transformed();

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
    // uncomment #include "xform.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "vector.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "line.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "plane.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "mesh.h"
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
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "mesh.h"
    // look_at_right_handed(eye, target, up) — view matrix from eye looking at target
    // Right-handed: X cross Y = Z (OpenGL/Vulkan). Camera looks down -Z.
    // Left-handed (DirectX) would look down +Z instead.
    // Not a projection — rigid transform (rotation + translation), no distortion.
    // up=(0,0,1) Z-up standard — any future viewer must use Z-up.
    // up must not be parallel to the view direction (eye->target).
    // Camera at (0,3,0) looking down at origin -> output = top view of the box.
    // Transform: (x,y,z) -> (-x, z, y-3)
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
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "mesh.h"
    // Identical to look_at_right_handed, only the input differs:
    // look_at: takes a target Point  — "look AT that point"
    // look_to: takes a direction Vector — "look TOWARD that direction"
    // look_to(eye, dir, up) == look_at(eye, eye+dir, up)
    // Right-handed: X cross Y = Z (OpenGL/Vulkan). Camera looks down -Z.
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
    // uncomment #include "xform.h"
    // uncomment #include "mesh.h"
    // perspective(fov_y, aspect, near, far) — projects 3D to clip space with foreshortening.
    // Objects farther away appear smaller. XY divided by depth (w = -z).
    // fov_y = vertical field of view. PI/2 = 90 degrees.
    // aspect = width/height. 1.0 = square viewport.
    // near/far = clipping planes. Objects between -near and -far in view space are visible.
    // Near face maps to z_ndc=0, far face maps to z_ndc=1.
    // Box translated to z=-2 so it sits between near=1 and far=3.
    // Near face (z=-1): stays full size +-1.
    // Far face (z=-3): shrinks to +-1/3 — this IS the perspective effect.
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
    // uncomment #include "xform.h"
    // uncomment #include "mesh.h"
    // orthographic(left, right, bottom, top, near, far) — parallel projection, no foreshortening.
    // Unlike perspective: far objects keep the SAME size as near objects.
    // Same box setup as perspective (translated to z=-2, between near=1 and far=3).
    // Near face (z=-1): +-1.  Far face (z=-3): ALSO +-1.  No shrinking!
    // This is the key difference: perspective shrinks far objects, orthographic does not.
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
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "plane.h"
    // uncomment #include "polyline.h"
    // project_to_plane(plane) — orthogonal projection onto a plane.
    // Every point drops perpendicularly onto the plane (along the normal).
    // XY plane at origin. Box lifted to z=1 so it hovers above the plane.
    // All z -> 0. Projected mesh is degenerate, so output as polyline outline.
    Plane plane(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0));
    Xform move = Xform::translation(0, 0, 1);
    Xform proj = Xform::project_to_plane(plane);
    Xform xf = proj * move;
    auto tp = [&](double x, double y, double z) { Point p(x,y,z); p.xform = xf; return p.transformed(); };
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
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "plane.h"
    // uncomment #include "polyline.h"
    // project_to_plane_by_axis(plane, direction) — projection along a direction onto a plane.
    // Unlike project_to_plane which drops perpendicular (along normal),
    // this projects along a chosen direction — like casting shadows from a light.
    // XY plane, direction (1,0,1) = diagonal in XZ. Box lifted to z=1.
    // Bottom face (z=0) stays put. Top face (z=2) slides along (1,0,1), shifting x by -2.
    // Convex outline of the projected shadow is a rectangle from x=-3 to x=1.
    Plane plane(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0));
    Vector direction(1, 0, 1);
    Xform move = Xform::translation(0, 0, 1);
    Xform proj = Xform::project_to_plane_by_axis(plane, direction);
    Xform xf = proj * move;
    auto tp = [&](double x, double y, double z) { Point p(x,y,z); p.xform = xf; return p.transformed(); };
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
    // uncomment #include "xform.h"
    // uncomment #include "mesh.h"
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
}

MINI_TEST("Xform", "Transform Geometry") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "line.h"
    // uncomment #include "plane.h"
    // uncomment #include "polyline.h"

    // Simple translation by (10, 20, 30)
    Xform t = Xform::translation(10.0, 20.0, 30.0);

    // Transform Point: (1,2,3) -> (11,22,33)
    Point pt(1.0, 2.0, 3.0);
    pt.xform = t;
    Point pt_transformed = pt.transformed();

    // Transform Vector: translation should NOT affect vectors
    Vector v(1.0, 0.0, 0.0);
    v.xform = t;
    Vector v_transformed = v.transformed();

    // Transform Line: (0,0,0)-(1,0,0) -> (10,20,30)-(11,20,30)
    Line ln(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    ln.xform = t;
    Line ln_transformed = ln.transformed();

    // Transform Plane: origin (0,0,0) -> (10,20,30)
    Point pl_o(0.0, 0.0, 0.0);
    Vector pl_x(1.0, 0.0, 0.0);
    Vector pl_y(0.0, 1.0, 0.0);
    Plane pl(pl_o, pl_x, pl_y);
    pl.xform = t;
    Plane pl_transformed = pl.transformed();

    // Transform Polyline: 3 points translated
    Polyline poly({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)});
    poly.xform = t;
    Polyline poly_transformed = poly.transformed();
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
    // uncomment #include "xform.h"
    Xform xform = Xform::translation(1.0, 2.0, 3.0);
    xform.name = "test_xform";

    std::string filename = "serialization/test_xform.json";
    xform.json_dump(filename);
    Xform loaded = Xform::json_load(filename);

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
    // uncomment #include "xform.h"
    Xform xform = Xform::translation(1.0, 2.0, 3.0);
    xform.name = "test_xform_proto";

    std::string filename = "serialization/test_xform.bin";
    xform.pb_dump(filename);
    Xform loaded = Xform::pb_load(filename);

    MINI_CHECK(loaded.name == "test_xform_proto");
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

} // namespace session_cpp
