#include "mini_test.h"
#include "plane.h"
#include "point.h"
#include "vector.h"
#include "tolerance.h"
#include "encoders.h"

#include <cmath>
#include <vector>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Plane", "constructor") {
    // uncomment #include "plane.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include <vector>

    // Default constructor - XY plane at origin
    Plane pl;

    // Origin and axes
    Point origin = pl.origin();
    Vector x_axis = pl.x_axis();
    Vector y_axis = pl.y_axis();
    Vector z_axis = pl.z_axis();

    // Plane equation coefficients (ax + by + cz + d = 0)
    double a = pl.a();
    double b = pl.b();
    double c = pl.c();
    double d = pl.d();

    // Index access for axes
    Vector ax0 = pl[0];
    Vector ax1 = pl[1];
    Vector ax2 = pl[2];

    // Minimal and Full String Representation
    std::string plstr = pl.str();
    std::string plrepr = pl.repr();

    // Copy (duplicates everything except guid)
    Plane plcopy = pl;

    // From point and normal
    Point p(0.0, 0.0, 5.0);
    Vector n(0.0, 0.0, 1.0);
    Plane pl_pn = Plane::from_point_normal(p, n);

    // From three points
    std::vector<Point> pts = {Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(0.0, 1.0, 0.0)};
    Plane pl_pts = Plane::from_points(pts);

    // From two points
    Point p1(0.0, 0.0, 0.0);
    Point p2(1.0, 0.0, 0.0);
    Plane pl_2pts = Plane::from_two_points(p1, p2);

    // Standard planes
    Plane xy = Plane::xy_plane();
    Plane yz = Plane::yz_plane();
    Plane xz = Plane::xz_plane();

    // Translation operators
    Vector offset(1.0, 2.0, 3.0);

    // In-place add
    Plane pl_iadd = Plane::xy_plane();
    pl_iadd += offset;

    // In-place subtract
    Plane pl_isub = Plane::xy_plane();
    pl_isub -= offset;

    // Copy add/subtract
    Plane pl_base = Plane::xy_plane();
    Plane pl_add = pl_base + offset;
    Plane pl_sub = pl_base - offset;

    MINI_CHECK(pl.name == "my_plane" && !pl.guid.empty());
    MINI_CHECK(TOLERANCE.is_close(origin[0], 0.0) && TOLERANCE.is_close(origin[1], 0.0) && TOLERANCE.is_close(origin[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(x_axis[0], 1.0) && TOLERANCE.is_close(x_axis[1], 0.0) && TOLERANCE.is_close(x_axis[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(y_axis[0], 0.0) && TOLERANCE.is_close(y_axis[1], 1.0) && TOLERANCE.is_close(y_axis[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(z_axis[0], 0.0) && TOLERANCE.is_close(z_axis[1], 0.0) && TOLERANCE.is_close(z_axis[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(a, 0.0) && TOLERANCE.is_close(b, 0.0) && TOLERANCE.is_close(c, 1.0) && TOLERANCE.is_close(d, 0.0));
    MINI_CHECK(TOLERANCE.is_close(ax0[0], 1.0) && TOLERANCE.is_close(ax1[1], 1.0) && TOLERANCE.is_close(ax2[2], 1.0));
    MINI_CHECK(plstr == "0.000000, 0.000000, 0.000000");
    MINI_CHECK(plrepr == "Plane(my_plane, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000)");
    MINI_CHECK(plcopy == pl && plcopy.guid != pl.guid);
    MINI_CHECK(TOLERANCE.is_close(pl_pn.origin()[2], 5.0) && TOLERANCE.is_close(pl_pn.z_axis()[2], 1.0) && TOLERANCE.is_close(pl_pn.d(), -5.0));
    MINI_CHECK(TOLERANCE.is_close(pl_pts.c(), 1.0) && TOLERANCE.is_close(pl_pts.d(), 0.0));
    MINI_CHECK(TOLERANCE.is_close(pl_2pts.origin()[0], 0.0) && TOLERANCE.is_close(pl_2pts.x_axis()[0], 1.0));
    MINI_CHECK(xy.name == "xy_plane" && TOLERANCE.is_close(xy.c(), 1.0));
    MINI_CHECK(yz.name == "yz_plane" && TOLERANCE.is_close(yz.a(), 1.0));
    MINI_CHECK(xz.name == "xz_plane" && TOLERANCE.is_close(xz.b(), 1.0));
    MINI_CHECK(TOLERANCE.is_close(pl_iadd.origin()[0], 1.0) && TOLERANCE.is_close(pl_iadd.origin()[1], 2.0) && TOLERANCE.is_close(pl_iadd.origin()[2], 3.0));
    MINI_CHECK(TOLERANCE.is_close(pl_isub.origin()[0], -1.0) && TOLERANCE.is_close(pl_isub.origin()[1], -2.0) && TOLERANCE.is_close(pl_isub.origin()[2], -3.0));
    MINI_CHECK(TOLERANCE.is_close(pl_add.origin()[2], 3.0) && TOLERANCE.is_close(pl_base.origin()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pl_sub.origin()[2], -3.0));
}

MINI_TEST("Plane", "reverse") {
    // uncomment #include "plane.h"

    // Reverse flips normal and swaps x/y axes
    Plane pl = Plane::xy_plane();
    pl.reverse();

    MINI_CHECK(TOLERANCE.is_close(pl.x_axis()[0], 0.0) && TOLERANCE.is_close(pl.x_axis()[1], 1.0) && TOLERANCE.is_close(pl.x_axis()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pl.y_axis()[0], 1.0) && TOLERANCE.is_close(pl.y_axis()[1], 0.0) && TOLERANCE.is_close(pl.y_axis()[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(pl.c(), -1.0));
}

MINI_TEST("Plane", "rotate") {
    // uncomment #include "plane.h"
    // uncomment #include "tolerance.h"

    // Rotate plane 90 degrees around its normal
    Plane pl = Plane::xy_plane();
    pl.rotate(Tolerance::PI / 2.0);

    MINI_CHECK(TOLERANCE.is_close(pl.x_axis()[1], 1.0));
}

MINI_TEST("Plane", "is_right_hand") {
    // uncomment #include "plane.h"
    // uncomment #include "tolerance.h"

    // All standard planes should be right-handed
    Plane xy = Plane::xy_plane();
    Plane yz = Plane::yz_plane();
    Plane xz = Plane::xz_plane();
    Plane default_pl;

    bool xy_rh = xy.is_right_hand();
    bool yz_rh = yz.is_right_hand();
    bool xz_rh = xz.is_right_hand();
    bool default_rh = default_pl.is_right_hand();

    // After reverse, should still be right-handed
    default_pl.reverse();
    bool reversed_rh = default_pl.is_right_hand();

    // After rotate, should still be right-handed
    default_pl.rotate(Tolerance::PI / 4.0);
    bool rotated_rh = default_pl.is_right_hand();

    MINI_CHECK(xy_rh == true);
    MINI_CHECK(yz_rh == true);
    MINI_CHECK(xz_rh == true);
    MINI_CHECK(default_rh == true);
    MINI_CHECK(reversed_rh == true);
    MINI_CHECK(rotated_rh == true);
}

MINI_TEST("Plane", "is_coplanar") {
    // uncomment #include "plane.h"
    // uncomment #include "vector.h"

    // Same direction (parallel planes)
    Plane p1 = Plane::xy_plane();
    Plane p2 = Plane::xy_plane();
    bool same_dir = Plane::is_same_direction(p1, p2, true);

    // Flipped direction
    Plane p3 = Plane::xy_plane();
    p3.reverse();
    bool same_dir_flipped = Plane::is_same_direction(p1, p3, true);
    bool same_dir_strict = Plane::is_same_direction(p1, p3, false);

    // Same position
    Plane p4 = Plane::xy_plane();
    bool same_pos = Plane::is_same_position(p1, p4);
    p4 += Vector(0.0, 0.0, 1.0);
    bool diff_pos = Plane::is_same_position(p1, p4);

    // Coplanar
    Plane p5 = Plane::xy_plane();
    Plane p6 = Plane::xy_plane();
    bool coplanar = Plane::is_coplanar(p5, p6, true);
    p6.reverse();
    bool coplanar_reversed = Plane::is_coplanar(p5, p6, true);
    p6 += Vector(0.0, 0.0, 1.0);
    bool not_coplanar = Plane::is_coplanar(p5, p6, true);

    MINI_CHECK(same_dir == true);
    MINI_CHECK(same_dir_flipped == true);
    MINI_CHECK(same_dir_strict == false);
    MINI_CHECK(same_pos == true);
    MINI_CHECK(diff_pos == false);
    MINI_CHECK(coplanar == true);
    MINI_CHECK(coplanar_reversed == true);
    MINI_CHECK(not_coplanar == false);
}

MINI_TEST("Plane", "transform") {
    // uncomment #include "plane.h"
    // uncomment #include "xform.h"

    // Transform - in-place transformation
    Plane pl = Plane::xy_plane();
    pl.xform = Xform::translation(1.0, 2.0, 3.0);
    pl.transform();

    // Transformed - returns new plane
    Plane pl2 = Plane::xy_plane();
    pl2.xform = Xform::translation(1.0, 2.0, 3.0);
    Plane pl3 = pl2.transformed();

    MINI_CHECK(TOLERANCE.is_close(pl.origin()[0], 1.0) && TOLERANCE.is_close(pl.origin()[1], 2.0) && TOLERANCE.is_close(pl.origin()[2], 3.0));
    MINI_CHECK(TOLERANCE.is_close(pl3.origin()[0], 1.0) && TOLERANCE.is_close(pl3.origin()[1], 2.0) && TOLERANCE.is_close(pl3.origin()[2], 3.0));
    MINI_CHECK(TOLERANCE.is_close(pl2.origin()[0], 0.0) && TOLERANCE.is_close(pl2.origin()[1], 0.0) && TOLERANCE.is_close(pl2.origin()[2], 0.0));
}

MINI_TEST("Plane", "json_roundtrip") {
    // uncomment #include "plane.h"

    Plane pl = Plane::xy_plane();
    pl.name = "test_plane";

    std::string fname = "test_plane.json";
    pl.json_dump(fname);
    Plane loaded = Plane::json_load(fname);

    MINI_CHECK(loaded.name == "test_plane");
    MINI_CHECK(TOLERANCE.is_close(loaded.c(), 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.d(), 0.0));
}

#ifdef ENABLE_PROTOBUF
MINI_TEST("Plane", "protobuf_roundtrip") {
    // uncomment #include "plane.h"

    Plane pl = Plane::xy_plane();
    pl.name = "test_plane";

    // protobuf_dump(fname) / protobuf_load(fname) - file-based serialization
    std::string fname = "test_plane.bin";
    pl.protobuf_dump(fname);
    Plane loaded = Plane::protobuf_load(fname);

    MINI_CHECK(loaded.name == "test_plane");
    MINI_CHECK(TOLERANCE.is_close(loaded.c(), 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.d(), 0.0));
}
#endif

} // namespace session_cpp
