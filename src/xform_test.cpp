#include "mini_test.h"
#include "xform.h"
#include "point.h"
#include "vector.h"
#include "mesh.h"
#include "line.h"
#include "plane.h"
#include "polyline.h"
#include "xform.pb.h"
#include "tolerance.h"
#include <array>
#include <string>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Xform", "Constructor") {
        // using session_cpp::Xform;
        // using session_cpp::Point;

        const Xform x;
        const double m00 = x.m[0];
        const double m11 = x.m[5];
        const double m22 = x.m[10];
        const double m33 = x.m[15];
        const bool is_id = x.is_identity();
        const Xform xfrom = Xform::from_matrix({
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            5.0, 10.0, 15.0, 1.0,
        });

        Xform xset;
        xset(1, 3) = 7.0;

        const std::string xstr = x.str();
        const std::string xrepr = x.repr();
        const Xform xcopy = x;
        const Xform xother;

        const Xform t = Xform::translation(10.0, 0.0, 0.0);
        const Xform s = Xform::scale_xyz(2.0, 1.0, 1.0);
        const Xform combined = t * s;
        const Point p(1.0, 0.0, 0.0);
        const Point result = p.transformed(combined);

        Xform t2 = Xform::translation(10.0, 0.0, 0.0);
        t2 *= s;
        const Point result2 = p.transformed(t2);

        MINI_CHECK(x.name == "my_xform");
        MINI_CHECK(!x.guid().empty());
        MINI_CHECK(m00 == 1.0 && m11 == 1.0 && m22 == 1.0 && m33 == 1.0);
        MINI_CHECK(is_id);
        MINI_CHECK(xfrom.m[12] == 5.0 && xfrom.m[13] == 10.0 && xfrom.m[14] == 15.0);
        MINI_CHECK(xfrom(0, 3) == 5.0 && xset.m[13] == 7.0);
        MINI_CHECK(xstr == "[1.000000, 0.000000, 0.000000, 0.000000]\n[0.000000, 1.000000, 0.000000, 0.000000]\n[0.000000, 0.000000, 1.000000, 0.000000]\n[0.000000, 0.000000, 0.000000, 1.000000]");
        MINI_CHECK(xrepr == "Xform(my_xform, " + x.guid().substr(0, 8) + ")");
        MINI_CHECK(xcopy == x && xcopy.guid() != x.guid());
        MINI_CHECK(xother == x);
        MINI_CHECK(xfrom != x);
        MINI_CHECK(result[0] == 12.0 && result[1] == 0.0 && result[2] == 0.0);
        MINI_CHECK(result2[0] == 12.0 && result2[1] == 0.0 && result2[2] == 0.0);
    }

    MINI_TEST("Xform", "From Axes") {
        // using session_cpp::Xform;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Xform xf = Xform::from_axes(Vector(0, 1, 0), Vector(-1, 0, 0), Vector(0, 0, 1));
        const Point p = Point(1, 2, 3).transformed(xf);

        MINI_CHECK(TOLERANCE.is_point_close(p, Point(-2, 1, 3)));
    }

    MINI_TEST("Xform", "Translation") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const Xform xf = Xform::translation(1.5, 1.0, 0.5);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(0.5, 0, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(2.5, 0, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(2.5, 2, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(0.5, 2, -0.5)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(0.5, 0, 1.5)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(2.5, 0, 1.5)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(2.5, 2, 1.5)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(0.5, 2, 1.5)));
    }

    MINI_TEST("Xform", "Rotation X") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const double s = std::sqrt(2.0);
        const Xform xf = Xform::rotation_x(Tolerance::PI / 4.0);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-1, 0, -s)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(1, 0, -s)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(1, s, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-1, s, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-1, -s, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(1, -s, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(1, 0, s)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(-1, 0, s)));
    }

    MINI_TEST("Xform", "Rotation Y") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const double s = std::sqrt(2.0);
        const Xform xf = Xform::rotation_y(Tolerance::PI / 4.0);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-s, -1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(0, -1, -s)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(0, 1, -s)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-s, 1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(0, -1, s)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(s, -1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(s, 1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(0, 1, s)));
    }

    MINI_TEST("Xform", "Rotation Z") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const double s = std::sqrt(2.0);
        const Xform xf = Xform::rotation_z(Tolerance::PI / 4.0);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(0, -s, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(s, 0, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(0, s, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-s, 0, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(0, -s, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(s, 0, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(0, s, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(-s, 0, 1)));
    }

    MINI_TEST("Xform", "Rotation Axis") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const double t = 1.0 / 3.0;
        const double k = 2.0 / std::sqrt(3.0);
        const Vector axis(1.0, 1.0, 1.0);
        const Xform xf = Xform::rotation(axis, 2.0 * Tolerance::PI / 4.0);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-1, -1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(-t, -t + k, -t - k)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(t - k, t + k, t)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-t - k, -t, -t + k)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-t + k, -t - k, -t)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(t + k, t, t - k)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(1, 1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(t, t - k, t + k)));
        MINI_CHECK(Xform::rotation(Vector::zero(), Tolerance::PI / 3.0).is_identity());
    }

    MINI_TEST("Xform", "Rotation Around Line") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::Line;

        const double s = std::sqrt(2.0);
        const Line line(-1.0, -1.0, -1.0, -1.0, -1.0, 1.0);
        const Xform xf = Xform::rotation_around_line(line, Tolerance::PI / 4.0);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-1, -1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(s - 1, s - 1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(-1, 2 * s - 1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-s - 1, s - 1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-1, -1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(s - 1, s - 1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(-1, 2 * s - 1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(-s - 1, s - 1, 1)));
    }

    MINI_TEST("Xform", "Change Basis") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Point o0(0, 0, 0);
        const Vector x0(1, 0, 0);
        const Vector y0(0, 1, 0);
        const Vector z0(0, 0, 1);
        const Point o1(0.5, -1.0, 0.5);
        const Vector x1(1.2, 0.0, 0.0);
        const Vector y1(0.3, -1.0, -0.15);
        const Vector z1(0.0, 0.0, 1.1);
        const Xform xf = Xform::change_basis(o0, x0, y0, z0, o1, x1, y1, z1);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-1, 0, -0.45)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(1.4, 0, -0.45)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(2, -2, -0.75)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-0.4, -2, -0.75)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-1, 0, 1.75)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(1.4, 0, 1.75)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(2, -2, 1.45)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(-0.4, -2, 1.45)));
    }

    MINI_TEST("Xform", "From Change Of Basis") {
        // using session_cpp::Xform;
        // using session_cpp::Point;
        // using session_cpp::Polyline;

        const Polyline rect0({
            Point(0.0, 0.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(2.0, 3.0, 0.0),
            Point(0.0, 3.0, 0.0),
        });
        const Polyline rect1({Point(0.0, 0.0, 4.0)});
        const Xform xf = Xform::from_change_of_basis(rect0, rect1);

        MINI_CHECK(TOLERANCE.is_close(xf.m[12], 1.0));
        MINI_CHECK(TOLERANCE.is_close(xf.m[13], 1.5));
        MINI_CHECK(TOLERANCE.is_close(xf.m[14], 2.0));
    }

    MINI_TEST("Xform", "Plane To Plane") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::Vector;
        // using session_cpp::Plane;

        const Plane pf(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0));
        const Plane pt(Point(2, 0, 0), Vector(0, 1, 0), Vector(-1, 0, 0));
        const Xform xf = Xform::plane_to_plane(pf, pt);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(1, 1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(1, -1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(3, -1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(3, 1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(1, 1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(1, -1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(3, -1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(3, 1, 1)));
    }

    MINI_TEST("Xform", "World To Frame") {
        // using session_cpp::Xform;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Point origin(1.0, 2.0, 3.0);
        const Vector x_axis(0.0, 1.0, 0.0);
        const Vector y_axis(0.0, 0.0, 1.0);
        const Vector z_axis(1.0, 0.0, 0.0);
        const Xform xf = Xform::world_to_frame(origin, x_axis, y_axis, z_axis);
        const Point p = Point(1.0, 4.0, 6.0).transformed(xf);

        MINI_CHECK(TOLERANCE.is_close(p[0], 2.0));
        MINI_CHECK(TOLERANCE.is_close(p[1], 3.0));
        MINI_CHECK(TOLERANCE.is_close(p[2], 0.0));
    }

    MINI_TEST("Xform", "Frame To World") {
        // using session_cpp::Xform;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Point origin(1.0, 2.0, 3.0);
        const Vector x_axis(0.0, 1.0, 0.0);
        const Vector y_axis(0.0, 0.0, 1.0);
        const Vector z_axis(1.0, 0.0, 0.0);
        const Xform xf = Xform::frame_to_world(origin, x_axis, y_axis, z_axis);
        const Point p = Point(2.0, 3.0, 0.0).transformed(xf);

        MINI_CHECK(TOLERANCE.is_close(p[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(p[1], 4.0));
        MINI_CHECK(TOLERANCE.is_close(p[2], 6.0));
    }

    MINI_TEST("Xform", "To Frame") {
        // using session_cpp::Xform;
        // using session_cpp::Plane;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Plane frame(Point(1, 2, 3), Vector(0, 1, 0), Vector(-1, 0, 0));
        const Xform xf = Xform::to_frame(frame);
        const Point px = Point(1, 0, 0).transformed(xf);
        const Point py = Point(0, 1, 0).transformed(xf);

        MINI_CHECK(TOLERANCE.is_point_close(px, Point(1, 3, 3)));
        MINI_CHECK(TOLERANCE.is_point_close(py, Point(0, 2, 3)));
    }

    MINI_TEST("Xform", "Scale XYZ") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const Xform xf = Xform::scale_xyz(1.5, 1.2, 1.8);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-1.5, -1.2, -1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(1.5, -1.2, -1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(1.5, 1.2, -1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-1.5, 1.2, -1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-1.5, -1.2, 1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(1.5, -1.2, 1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(1.5, 1.2, 1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(-1.5, 1.2, 1.8)));
    }

    MINI_TEST("Xform", "Scale Uniform") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const Point c(0, 0, 0);
        const Xform xf = Xform::scale_uniform(c, 2.0);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-2, -2, -2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(2, -2, -2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(2, 2, -2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-2, 2, -2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-2, -2, 2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(2, -2, 2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(2, 2, 2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(-2, 2, 2)));
    }

    MINI_TEST("Xform", "Scale Non Uniform") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const Point c(0, 0, 0);
        const Xform xf = Xform::scale_non_uniform(c, 1.5, 1.2, 1.8);
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-1.5, -1.2, -1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(1.5, -1.2, -1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(1.5, 1.2, -1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-1.5, 1.2, -1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-1.5, -1.2, 1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(1.5, -1.2, 1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(1.5, 1.2, 1.8)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(-1.5, 1.2, 1.8)));
    }

    MINI_TEST("Xform", "Axis Rotation") {
        // using session_cpp::Xform;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Xform xf = Xform::axis_rotation(90.0, Vector(0, 0, 1), true);
        const Point p = Point(1, 0, 0).transformed(xf);

        MINI_CHECK(TOLERANCE.is_point_close(p, Point(0, 1, 0)));
    }

    MINI_TEST("Xform", "Look At Right Handed") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Point eye(0, 3, 0);
        const Point target(0, 0, 0);
        const Xform xf = Xform::look_at_right_handed(eye, target, Vector(0, 0, 1));
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(1, -1, -4)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(-1, -1, -4)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(-1, -1, -2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(1, -1, -2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(1, 1, -4)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(-1, 1, -4)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(-1, 1, -2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(1, 1, -2)));
    }

    MINI_TEST("Xform", "Look To Right Handed") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Point eye(0, 3, 0);
        const Vector direction(0, -1, 0);
        const Xform xf = Xform::look_to_right_handed(eye, direction, Vector(0, 0, 1));
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(1, -1, -4)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(-1, -1, -4)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(-1, -1, -2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(1, -1, -2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(1, 1, -4)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(-1, 1, -4)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(-1, 1, -2)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(1, 1, -2)));
    }

    MINI_TEST("Xform", "Perspective") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const double t = 1.0 / 3.0;
        const Xform view = Xform::translation(0, 0, -2);
        const Xform proj = Xform::perspective(Tolerance::PI / 2.0, 1.0, 1.0, 3.0);
        const Xform xf = proj * view;
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-t, -t, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(t, -t, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(t, t, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-t, t, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-1, -1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(1, -1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(1, 1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(-1, 1, 0)));
    }

    MINI_TEST("Xform", "Orthographic") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const Xform view = Xform::translation(0, 0, -2);
        const Xform proj = Xform::orthographic(-1.0, 1.0, -1.0, 1.0, 1.0, 3.0);
        const Xform xf = proj * view;
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(xf).to_vertices_and_faces().first;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-1, -1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(1, -1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(1, 1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-1, 1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-1, -1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(1, -1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(1, 1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(-1, 1, 0)));
    }

    MINI_TEST("Xform", "Project To Plane") {
        // using session_cpp::Xform;
        // using session_cpp::Point;
        // using session_cpp::Vector;
        // using session_cpp::Plane;
        // using session_cpp::Polyline;

        const Plane plane(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0));
        const Xform shift = Xform::translation(0, 0, 1);
        const Xform proj = Xform::project_to_plane(plane);
        const Xform xf = proj * shift;
        const Polyline outline({
            Point(-1, -1, -1).transformed(xf),
            Point(1, -1, -1).transformed(xf),
            Point(1, 1, -1).transformed(xf),
            Point(-1, 1, -1).transformed(xf),
            Point(-1, -1, -1).transformed(xf),
        });
        const std::vector<Point> points = outline.get_points();

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-1, -1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(1, -1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(1, 1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-1, 1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-1, -1, 0)));
    }

    MINI_TEST("Xform", "Project To Plane By Axis") {
        // using session_cpp::Xform;
        // using session_cpp::Point;
        // using session_cpp::Vector;
        // using session_cpp::Plane;
        // using session_cpp::Polyline;

        const Plane plane(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0));
        const Vector direction(1, 0, 1);
        const Xform shift = Xform::translation(0, 0, 1);
        const Xform proj = Xform::project_to_plane_by_axis(plane, direction);
        const Xform xf = proj * shift;
        const Polyline outline({
            Point(-1, -1, 1).transformed(xf),
            Point(1, -1, -1).transformed(xf),
            Point(1, 1, -1).transformed(xf),
            Point(-1, 1, 1).transformed(xf),
            Point(-1, -1, 1).transformed(xf),
        });
        const std::vector<Point> points = outline.get_points();

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-3, -1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(1, -1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(1, 1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-3, 1, 0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-3, -1, 0)));
    }

    MINI_TEST("Xform", "Transform Point") {
        // using session_cpp::Xform;
        // using session_cpp::Point;

        const Xform t = Xform::translation(10.0, 20.0, 30.0);
        const Xform s = Xform::scale_xyz(2.0, 3.0, 4.0);
        const Xform composite = t * s;
        const Point p = composite.transform_point(Point(1.0, 1.0, 1.0));

        Xform pr = Xform::identity();
        pr.m[0] = 1.2;
        pr.m[5] = 0.8;
        pr.m[10] = 1.1;
        pr.m[14] = 0.5;
        pr.m[11] = -1.0;
        pr.m[15] = 0.0;
        const Point q = pr.transform_point(Point(1.0, 1.0, 2.0));

        MINI_CHECK(TOLERANCE.is_point_close(p, Point(12.0, 23.0, 34.0)));
        MINI_CHECK(TOLERANCE.is_point_close(q, Point(-0.6, -0.4, -1.35)));
    }

    MINI_TEST("Xform", "Transform Vector") {
        // using session_cpp::Xform;
        // using session_cpp::Vector;

        const Xform t = Xform::translation(10.0, 20.0, 30.0);
        const Xform s = Xform::scale_xyz(2.0, 3.0, 4.0);
        const Xform composite = t * s;
        const Vector v = composite.transform_vector(Vector(1.0, 1.0, 1.0));
        const Xform r = Xform::rotation_z(90.0, true);
        const Vector u = r.transform_vector(Vector::x_axis());

        MINI_CHECK(TOLERANCE.is_vector_close(v, Vector(2.0, 3.0, 4.0)));
        MINI_CHECK(TOLERANCE.is_vector_close(u, Vector::y_axis()));
    }

    MINI_TEST("Xform", "Transform Geometry") {
        // using session_cpp::Xform;
        // using session_cpp::Point;
        // using session_cpp::Vector;
        // using session_cpp::Line;
        // using session_cpp::Plane;
        // using session_cpp::Polyline;

        const Xform t = Xform::translation(10.0, 20.0, 30.0);
        const Point pt(1.0, 2.0, 3.0);
        const Point pt_transformed = pt.transformed(t);
        const Vector v(1.0, 0.0, 0.0);
        const Vector v_transformed = v.transformed(t);
        const Line ln(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        const Line ln_transformed = ln.transformed(t);
        const Plane pl(Point(0.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
        const Plane pl_transformed = pl.transformed(t);
        const Polyline poly({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)});
        const std::vector<Point> points = poly.transformed(t).get_points();

        MINI_CHECK(TOLERANCE.is_point_close(pt_transformed, Point(11.0, 22.0, 33.0)));
        MINI_CHECK(v_transformed[0] == 1.0 && v_transformed[1] == 0.0 && v_transformed[2] == 0.0);
        MINI_CHECK(ln_transformed[0] == 10.0 && ln_transformed[1] == 20.0 && ln_transformed[2] == 30.0);
        MINI_CHECK(ln_transformed[3] == 11.0 && ln_transformed[4] == 20.0 && ln_transformed[5] == 30.0);
        MINI_CHECK(TOLERANCE.is_point_close(pl_transformed.origin(), Point(10.0, 20.0, 30.0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(10.0, 20.0, 30.0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(11.0, 20.0, 30.0)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(11.0, 21.0, 30.0)));
    }

    MINI_TEST("Xform", "Inverse") {
        // using session_cpp::Xform;
        // using session_cpp::Mesh;
        // using session_cpp::Point;

        const Xform t = Xform::translation(1.0, 0.5, 0.5);
        const Xform s = Xform::scale_xyz(1.5, 1.2, 1.3);
        const Xform composite = t * s;
        const Xform inv = composite.inverse().value();
        const Mesh mesh = Mesh::create_box(2, 2, 2);
        const std::vector<Point> points = mesh.transformed(composite).transformed(inv).to_vertices_and_faces().first;

        Xform p = Xform::identity();
        p.m[0] = 1.2;
        p.m[5] = 0.8;
        p.m[10] = 1.1;
        p.m[14] = 0.5;
        p.m[11] = -1.0;
        p.m[15] = 0.0;
        const Xform pinv = p.inverse().value();
        const Xform prod = p * pinv;

        MINI_CHECK(TOLERANCE.is_point_close(points[0], Point(-1, -1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[1], Point(1, -1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[2], Point(1, 1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[3], Point(-1, 1, -1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[4], Point(-1, -1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[5], Point(1, -1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[6], Point(1, 1, 1)));
        MINI_CHECK(TOLERANCE.is_point_close(points[7], Point(-1, 1, 1)));
        MINI_CHECK(prod.is_identity());
    }

    MINI_TEST("Xform", "To Cols") {
        // using session_cpp::Xform;

        const Xform xf = Xform::translation(1.0, 2.0, 3.0);
        const std::array<std::array<double, 4>, 4> cols = xf.to_cols();

        MINI_CHECK(TOLERANCE.is_close(cols[0][0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(cols[1][1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(cols[2][2], 1.0));
        MINI_CHECK(TOLERANCE.is_close(cols[3][3], 1.0));
        MINI_CHECK(TOLERANCE.is_close(cols[3][0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(cols[3][1], 2.0));
        MINI_CHECK(TOLERANCE.is_close(cols[3][2], 3.0));
    }

    MINI_TEST("Xform", "Uniform Scale") {
        // using session_cpp::Xform;

        MINI_CHECK(TOLERANCE.is_close(Xform::scale_xyz(2.0, 2.0, 2.0).uniform_scale(), 2.0));
        MINI_CHECK(TOLERANCE.is_close(Xform::translation(1.0, 2.0, 3.0).uniform_scale(), 1.0));
    }

    MINI_TEST("Xform", "Eye") {
        // using session_cpp::Xform;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Xform view = Xform::look_at_right_handed(Point(1.0, 2.0, 5.0), Point(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
        const Xform perspective = Xform::perspective(Tolerance::PI / 2.0, 1.0, 1.0, 10.0) * view;
        const Xform orthographic = Xform::orthographic(-2.0, 2.0, -1.0, 1.0, 1.0, 10.0) * view;

        MINI_CHECK(TOLERANCE.is_point_close(perspective.eye(), Point(1.0, 2.0, 5.0)));
        MINI_CHECK(orthographic.eye().distance(Point(0.0, 0.0, 0.0)) > 1.0e8);
    }

    MINI_TEST("Xform", "Ortho Half Height") {
        // using session_cpp::Xform;
        // using session_cpp::Point;
        // using session_cpp::Vector;

        const Xform view = Xform::look_at_right_handed(Point(1.0, 2.0, 5.0), Point(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
        const Xform perspective = Xform::perspective(Tolerance::PI / 2.0, 1.0, 1.0, 10.0) * view;
        const Xform orthographic = Xform::orthographic(-2.0, 2.0, -1.0, 1.0, 1.0, 10.0) * view;

        MINI_CHECK(TOLERANCE.is_close(perspective.ortho_half_height(), 0.0));
        MINI_CHECK(TOLERANCE.is_close(orthographic.ortho_half_height(), 1.0));
    }

    MINI_TEST("Xform", "Json Roundtrip") {
        // using session_cpp::Xform;

        Xform xform = Xform::translation(1.0, 2.0, 3.0);
        xform.name = "test_xform";

        const std::string filename = "serialization/test_xform.json";
        xform.file_json_dump(filename);

        const Xform loaded = Xform::file_json_load(filename);
        const Xform parsed = Xform::file_json_loads(xform.file_json_dumps());

        MINI_CHECK(loaded.name == "test_xform");
        MINI_CHECK(loaded.guid() == xform.guid());
        MINI_CHECK(loaded == xform);
        MINI_CHECK(parsed == xform && parsed.guid() == xform.guid());
        MINI_CHECK(TOLERANCE.is_close(loaded.m[12], 1.0) && TOLERANCE.is_close(loaded.m[13], 2.0));
        MINI_CHECK(TOLERANCE.is_close(loaded.m[14], 3.0) && TOLERANCE.is_close(loaded.m[15], 1.0));
    }

    MINI_TEST("Xform", "Protobuf Roundtrip") {
        // using session_cpp::Xform;

        Xform xform = Xform::translation(1.0, 2.0, 3.0);
        xform.name = "test_xform_proto";

        const std::string guid = xform.guid();
        const std::string filename = "serialization/test_xform.bin";
        xform.pb_dump(filename);

        const Xform loaded = Xform::pb_load(filename);
        const Xform converted = Xform::from_proto(xform.to_proto());

        MINI_CHECK(loaded.name == "test_xform_proto");
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(loaded == xform);
        MINI_CHECK(converted == xform && converted.guid() == guid);
        MINI_CHECK(TOLERANCE.is_close(loaded.m[12], 1.0) && TOLERANCE.is_close(loaded.m[13], 2.0));
        MINI_CHECK(TOLERANCE.is_close(loaded.m[14], 3.0) && TOLERANCE.is_close(loaded.m[15], 1.0));
    }

} // namespace session_cpp
