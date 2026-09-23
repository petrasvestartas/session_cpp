#include "mini_test.h"
#include "plane.h"
#include "plane.pb.h"
#include "point.h"
#include "tolerance.h"
#include "vector.h"
#include "xform.h"
#include <cmath>
#include <string>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Plane", "Constructor") {

        const Plane pl;

        const Point origin = pl.origin();
        const Vector x_axis = pl.x_axis();
        const Vector y_axis = pl.y_axis();
        const Vector z_axis = pl.z_axis();

        const double a = pl.a();
        const double b = pl.b();
        const double c = pl.c();
        const double d = pl.d();

        const Vector ax0 = pl[0];
        const Vector ax1 = pl[1];
        const Vector ax2 = pl[2];

        const std::string plstr = pl.str();
        const std::string plrepr = pl.repr();

        const Plane plcopy = pl;

        const Point p(0.0, 0.0, 5.0);
        const Vector n(0.0, 0.0, 1.0);
        const Plane pl_pn = Plane::from_point_normal(p, n);

        const std::vector<Point> pts = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(0.0, 1.0, 0.0),
        };
        const Plane pl_pts = Plane::from_points(pts);

        const Point p1(0.0, 0.0, 0.0);
        const Point p2(1.0, 0.0, 0.0);
        const Plane pl_2pts = Plane::from_two_points(p1, p2);

        const Plane xy = Plane::xy_plane();
        const Plane yz = Plane::yz_plane();
        const Plane xz = Plane::xz_plane();

        const Vector offset(1.0, 2.0, 3.0);
        Plane pl_iadd = Plane::xy_plane();
        pl_iadd += offset;

        Plane pl_isub = Plane::xy_plane();
        pl_isub -= offset;

        const Plane pl_base = Plane::xy_plane();
        const Plane pl_add = pl_base + offset;
        const Plane pl_sub = pl_base - offset;

        MINI_CHECK(pl.name == "my_plane" && !pl.guid().empty());
        MINI_CHECK(TOLERANCE.is_close(origin[0], 0.0) && TOLERANCE.is_close(origin[1], 0.0) && TOLERANCE.is_close(origin[2], 0.0));
        MINI_CHECK(TOLERANCE.is_close(x_axis[0], 1.0) && TOLERANCE.is_close(y_axis[1], 1.0) && TOLERANCE.is_close(z_axis[2], 1.0));
        MINI_CHECK(TOLERANCE.is_close(a, 0.0) && TOLERANCE.is_close(b, 0.0) && TOLERANCE.is_close(c, 1.0) && TOLERANCE.is_close(d, 0.0));
        MINI_CHECK(TOLERANCE.is_close(ax0[0], 1.0) && TOLERANCE.is_close(ax1[1], 1.0) && TOLERANCE.is_close(ax2[2], 1.0));
        MINI_CHECK(plstr == "0.000000, 0.000000, 0.000000\n1.000000, 0.000000, 0.000000\n0.000000, 1.000000, 0.000000\n0.000000, 0.000000, 1.000000");
        MINI_CHECK(plrepr == "Plane(my_plane, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, Color(blue, 0.0, 0.0, 1.0, 1.0))");
        MINI_CHECK(plcopy == pl && plcopy.guid() != pl.guid());
        MINI_CHECK(xy != yz);
        MINI_CHECK(TOLERANCE.is_close(pl_pn.origin()[2], 5.0) && TOLERANCE.is_close(pl_pn.z_axis()[2], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pl_pts.c(), 1.0));
        MINI_CHECK(TOLERANCE.is_close(pl_2pts.x_axis()[0], 1.0));
        MINI_CHECK(xy.name == "xy_plane" && yz.name == "yz_plane" && xz.name == "xz_plane");
        MINI_CHECK(TOLERANCE.is_close(pl_iadd.origin()[0], 1.0) && TOLERANCE.is_close(pl_iadd.origin()[1], 2.0) && TOLERANCE.is_close(pl_iadd.origin()[2], 3.0));
        MINI_CHECK(TOLERANCE.is_close(pl_isub.origin()[0], -1.0) && TOLERANCE.is_close(pl_isub.origin()[2], -3.0));
        MINI_CHECK(TOLERANCE.is_close(pl_add.origin()[2], 3.0));
        MINI_CHECK(TOLERANCE.is_close(pl_sub.origin()[2], -3.0));
    }

    MINI_TEST("Plane", "Is Valid") {

        const Plane pl = Plane::xy_plane();
        const Plane invalid = Plane::invalid();

        MINI_CHECK(pl.is_valid());
        MINI_CHECK(!invalid.is_valid());
    }

    MINI_TEST("Plane", "Reverse") {

        Plane pl = Plane::xy_plane();
        pl.reverse();

        MINI_CHECK(TOLERANCE.is_close(pl.x_axis()[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pl.x_axis()[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pl.y_axis()[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pl.y_axis()[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(pl.c(), -1.0));
    }

    MINI_TEST("Plane", "Rotate") {

        Plane pl = Plane::xy_plane();
        pl.rotate(Tolerance::PI / 2.0);

        MINI_CHECK(TOLERANCE.is_close(pl.x_axis()[1], 1.0));
    }

    MINI_TEST("Plane", "Is Right Hand") {

        const Plane xy = Plane::xy_plane();
        const Plane yz = Plane::yz_plane();
        const Plane xz = Plane::xz_plane();

        MINI_CHECK(xy.is_right_hand());
        MINI_CHECK(yz.is_right_hand());
        MINI_CHECK(xz.is_right_hand());
    }

    MINI_TEST("Plane", "Is Same Direction") {

        const Plane p1 = Plane::xy_plane();
        const Plane p2 = Plane::xy_plane();
        Plane p3 = Plane::xy_plane();
        p3.reverse();

        MINI_CHECK(Plane::is_same_direction(p1, p2, true));
        MINI_CHECK(Plane::is_same_direction(p1, p3, true));
        MINI_CHECK(Plane::is_same_direction(p1, p3, false));
    }

    MINI_TEST("Plane", "Is Same Position") {

        const Plane p1 = Plane::xy_plane();
        Plane p2 = Plane::xy_plane();
        p2 += Vector(0.0, 0.0, 1.0);

        MINI_CHECK(Plane::is_same_position(p1, Plane::xy_plane()));
        MINI_CHECK(!Plane::is_same_position(p1, p2));
    }

    MINI_TEST("Plane", "Is Coplanar") {

        const Plane p1 = Plane::xy_plane();
        const Plane p2 = Plane::xy_plane();
        Plane p3 = Plane::xy_plane();
        p3 += Vector(0.0, 0.0, 1.0);

        MINI_CHECK(Plane::is_coplanar(p1, p2, true));
        MINI_CHECK(!Plane::is_coplanar(p1, p3, true));
    }

    MINI_TEST("Plane", "Translate By Normal") {

        const Plane pl = Plane::xy_plane();
        const Plane moved = pl.translate_by_normal(5.0);

        MINI_CHECK(TOLERANCE.is_close(moved.origin()[2], 5.0));
        MINI_CHECK(TOLERANCE.is_close(pl.origin()[2], 0.0));
    }

    MINI_TEST("Plane", "Base1 Base2") {

        const Plane xy = Plane::xy_plane();
        const Vector b1 = xy.base1();
        const Vector b2 = xy.base2();

        MINI_CHECK(TOLERANCE.is_close(std::abs(b1.dot(xy.z_axis())), 0.0));
        MINI_CHECK(TOLERANCE.is_close(std::abs(b2.dot(xy.z_axis())), 0.0));
        MINI_CHECK(TOLERANCE.is_close(b1.dot(b2), 0.0));
        MINI_CHECK(TOLERANCE.is_close(b1.magnitude(), 1.0));
        MINI_CHECK(TOLERANCE.is_close(b2.magnitude(), 1.0));
    }

    MINI_TEST("Plane", "Transform") {

        Plane pl = Plane::xy_plane();
        const Xform pl_xf = Xform::translation(1.0, 2.0, 3.0);
        pl.transform(pl_xf);

        MINI_CHECK(TOLERANCE.is_close(pl.origin()[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pl.origin()[1], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pl.origin()[2], 3.0));
    }

    MINI_TEST("Plane", "Transformed") {

        const Plane pl = Plane::xy_plane();
        const Xform pl_xf = Xform::translation(1.0, 2.0, 3.0);
        const Plane pl2 = pl.transformed(pl_xf);

        MINI_CHECK(TOLERANCE.is_close(pl2.origin()[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(pl2.origin()[1], 2.0));
        MINI_CHECK(TOLERANCE.is_close(pl2.origin()[2], 3.0));
        MINI_CHECK(TOLERANCE.is_close(pl.origin()[0], 0.0));
    }

    MINI_TEST("Plane", "Json Roundtrip") {

        Plane pl = Plane::xy_plane();
        pl.name = "test_plane";

        const std::string guid = pl.guid();
        const std::string fname = "serialization/test_plane.json";
        pl.file_json_dump(fname);

        const Plane loaded = Plane::file_json_load(fname);
        const Plane parsed = Plane::file_json_loads(pl.file_json_dumps());

        MINI_CHECK(loaded.name == "test_plane");
        MINI_CHECK(TOLERANCE.is_close(loaded.c(), 1.0));
        MINI_CHECK(parsed == pl);
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(parsed.guid() == guid);
    }

    MINI_TEST("Plane", "Protobuf Roundtrip") {

        const Plane fresh;
        const session_proto::Plane fresh_proto = fresh.to_proto();
        Plane pl = Plane::xy_plane();
        pl.name = "test_plane";

        const std::string guid = pl.guid();
        const std::string fname = "serialization/test_plane.bin";
        pl.pb_dump(fname);

        const Plane loaded = Plane::pb_load(fname);
        const Plane parsed = Plane::pb_loads(pl.pb_dumps());
        const Plane converted = Plane::from_proto(pl.to_proto());

        MINI_CHECK(!fresh.has_guid());
        MINI_CHECK(fresh_proto.guid().empty());
        MINI_CHECK(loaded.name == "test_plane");
        MINI_CHECK(TOLERANCE.is_close(loaded.c(), 1.0));
        MINI_CHECK(parsed == pl);
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(parsed.guid() == guid);
        MINI_CHECK(converted == pl);
        MINI_CHECK(converted.guid() == guid);
    }

    MINI_TEST("Plane", "Has On Negative Side") {

        const Plane pl = Plane::xy_plane();
        const Point above(0.0, 0.0, 1.0);
        const Point below(0.0, 0.0, -1.0);

        MINI_CHECK(pl.has_on_negative_side(below));
        MINI_CHECK(!pl.has_on_negative_side(above));
    }

    MINI_TEST("Plane", "Squared Distance") {

        const Plane pl = Plane::xy_plane();
        const Point above(1.0, 2.0, 3.0);
        const Point on(4.0, 5.0, 0.0);

        MINI_CHECK(TOLERANCE.is_close(pl.squared_distance(above), 9.0));
        MINI_CHECK(TOLERANCE.is_close(pl.squared_distance(on), 0.0));
    }

    MINI_TEST("Plane", "Axis Point") {

        const Plane pl = Plane::from_point_normal(Point(1.0, 2.0, 3.0), Vector(0.0, 0.0, 1.0));
        const Point p = pl.axis_point();

        MINI_CHECK(TOLERANCE.is_close(p[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(p[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(p[2], 3.0));
        MINI_CHECK(TOLERANCE.is_close(pl.squared_distance(p), 0.0));
    }

} // namespace session_cpp
