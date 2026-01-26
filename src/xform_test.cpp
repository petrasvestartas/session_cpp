#include "mini_test.h"
#include "xform.h"
#include "point.h"
#include "vector.h"
#include "line.h"
#include "plane.h"
#include "polyline.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Xform", "constructor") {
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
    Xform xfrom = Xform::from_matrix({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 10.0, 15.0, 1.0});

    // Minimal and Full String Representation
    std::string xstr = x.str();
    std::string xrepr = x.repr();

    // Copy (duplicates everything except guid)
    Xform xcopy = x;
    Xform xother;

    // Equality operators
    bool x_eq = (x == xother);
    bool x_ne = (x != xfrom);

    // Matrix multiplication (*)
    Xform t = Xform::translation(10.0, 0.0, 0.0);
    Xform s = Xform::scaling(2.0, 1.0, 1.0);
    Xform combined = t * s;
    Point p(1.0, 0.0, 0.0);
    Point result = combined.transformed_point(p);

    // In-place multiplication (*=)
    Xform t2 = Xform::translation(10.0, 0.0, 0.0);
    t2 *= s;
    Point result2 = t2.transformed_point(p);

    MINI_CHECK(x.name == "my_xform" && !x.guid.empty());
    MINI_CHECK(m00 == 1.0 && m11 == 1.0 && m22 == 1.0 && m33 == 1.0);
    MINI_CHECK(is_id == true);
    MINI_CHECK(xfrom.m[12] == 5.0 && xfrom.m[13] == 10.0 && xfrom.m[14] == 15.0);
    MINI_CHECK(xstr.find("1.000000") != std::string::npos);
    MINI_CHECK(xrepr.find("Xform(") != std::string::npos && xrepr.find("my_xform") != std::string::npos);
    MINI_CHECK(xcopy == x && xcopy.guid != x.guid);
    MINI_CHECK(x_eq == true && x_ne == true);
    // (1,0,0) * scale(2,1,1) = (2,0,0), then translate(10,0,0) = (12,0,0)
    MINI_CHECK(TOLERANCE.is_close(result[0], 12.0) && TOLERANCE.is_close(result[1], 0.0) && TOLERANCE.is_close(result[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(result2[0], 12.0) && TOLERANCE.is_close(result2[1], 0.0) && TOLERANCE.is_close(result2[2], 0.0));
}

MINI_TEST("Xform", "translation") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"

    // Translation matrix
    Xform t = Xform::translation(1.0, 2.0, 3.0);

    // Apply to point
    Point p(4.0, 5.0, 6.0);
    Point tp = t.transformed_point(p);

    MINI_CHECK(TOLERANCE.is_close(tp[0], 5.0));
    MINI_CHECK(TOLERANCE.is_close(tp[1], 7.0));
    MINI_CHECK(TOLERANCE.is_close(tp[2], 9.0));
}

MINI_TEST("Xform", "scaling") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"

    // Scaling matrix
    Xform s = Xform::scaling(2.0, 3.0, 4.0);

    // Apply to point
    Point p(1.0, 1.0, 1.0);
    Point sp = s.transformed_point(p);

    MINI_CHECK(TOLERANCE.is_close(sp[0], 2.0));
    MINI_CHECK(TOLERANCE.is_close(sp[1], 3.0));
    MINI_CHECK(TOLERANCE.is_close(sp[2], 4.0));
}

MINI_TEST("Xform", "rotation") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "tolerance.h"

    // Rotation around X axis by 90 degrees
    Xform rx = Xform::rotation_x(Tolerance::PI / 2.0);
    // Apply to point (0,1,0) -> (0,0,1)
    Point px(0.0, 1.0, 0.0);
    Point rpx = rx.transformed_point(px);

    // Rotation around Y axis by 90 degrees
    Xform ry = Xform::rotation_y(Tolerance::PI / 2.0);
    // Apply to point (0,0,1) -> (1,0,0)
    Point py(0.0, 0.0, 1.0);
    Point rpy = ry.transformed_point(py);

    // Rotation around Z axis by 90 degrees
    Xform rz = Xform::rotation_z(Tolerance::PI / 2.0);
    // Apply to point (1,0,0) -> (0,1,0)
    Point pz(1.0, 0.0, 0.0);
    Point rpz = rz.transformed_point(pz);

    // Rotation around arbitrary axis (1,1,1) by 120 degrees
    // This cycles x->y->z->x
    Vector axis(1.0, 1.0, 1.0);
    Xform r = Xform::rotation(axis, 2.0 * Tolerance::PI / 3.0);
    // Apply to point (1,0,0) -> (0,1,0)
    Point p(1.0, 0.0, 0.0);
    Point rp = r.transformed_point(p);

    MINI_CHECK(TOLERANCE.is_close(rpx[0], 0.0) && TOLERANCE.is_close(rpx[1], 0.0) && TOLERANCE.is_close(rpx[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(rpy[0], 1.0) && TOLERANCE.is_close(rpy[1], 0.0) && TOLERANCE.is_close(rpy[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(rpz[0], 0.0) && TOLERANCE.is_close(rpz[1], 1.0) && TOLERANCE.is_close(rpz[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(rp[0], 0.0) && TOLERANCE.is_close(rp[1], 1.0) && TOLERANCE.is_close(rp[2], 0.0));
}

MINI_TEST("Xform", "inverse") {
    // uncomment #include "xform.h"

    // Create composite transformation
    Xform t = Xform::translation(1.0, 2.0, 3.0);
    Xform s = Xform::scaling(2.0, 2.0, 2.0);
    Xform composite = t * s;

    // Compute inverse
    auto inv_opt = composite.inverse();
    Xform inv = inv_opt.value();

    // Multiply should give identity
    Xform result = composite * inv;

    MINI_CHECK(result.is_identity());
}

MINI_TEST("Xform", "transform_geometry") {
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
    Vector v_transformed = t.transformed_vector(v);

    // Transform Line: (0,0,0)-(1,0,0) -> (10,20,30)-(11,20,30)
    Line ln(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    ln.xform = t;
    Line ln_transformed = ln.transformed();

    // Transform Plane: origin (0,0,0) -> (10,20,30)
    Point pl_origin(0.0, 0.0, 0.0);
    Vector pl_x(1.0, 0.0, 0.0);
    Vector pl_y(0.0, 1.0, 0.0);
    Plane pl(pl_origin, pl_x, pl_y);
    pl.xform = t;
    Plane pl_transformed = pl.transformed();

    // Transform Polyline: 3 points translated
    Polyline poly({Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)});
    poly.xform = t;
    Polyline poly_transformed = poly.transformed();
    std::vector<Point> pts = poly_transformed.get_points();

    MINI_CHECK(TOLERANCE.is_close(pt_transformed[0], 11.0) && TOLERANCE.is_close(pt_transformed[1], 22.0) && TOLERANCE.is_close(pt_transformed[2], 33.0));
    MINI_CHECK(TOLERANCE.is_close(v_transformed[0], 1.0) && TOLERANCE.is_close(v_transformed[1], 0.0) && TOLERANCE.is_close(v_transformed[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(ln_transformed[0], 10.0) && TOLERANCE.is_close(ln_transformed[1], 20.0) && TOLERANCE.is_close(ln_transformed[2], 30.0));
    MINI_CHECK(TOLERANCE.is_close(ln_transformed[3], 11.0) && TOLERANCE.is_close(ln_transformed[4], 20.0) && TOLERANCE.is_close(ln_transformed[5], 30.0));
    MINI_CHECK(TOLERANCE.is_close(pl_transformed.origin()[0], 10.0) && TOLERANCE.is_close(pl_transformed.origin()[1], 20.0) && TOLERANCE.is_close(pl_transformed.origin()[2], 30.0));
    MINI_CHECK(TOLERANCE.is_close(pts[0][0], 10.0) && TOLERANCE.is_close(pts[0][1], 20.0) && TOLERANCE.is_close(pts[0][2], 30.0));
    MINI_CHECK(TOLERANCE.is_close(pts[1][0], 11.0) && TOLERANCE.is_close(pts[1][1], 20.0) && TOLERANCE.is_close(pts[1][2], 30.0));
    MINI_CHECK(TOLERANCE.is_close(pts[2][0], 11.0) && TOLERANCE.is_close(pts[2][1], 21.0) && TOLERANCE.is_close(pts[2][2], 30.0));
}

MINI_TEST("Xform", "change_basis") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"

    // System 0: standard XY plane at origin
    Point origin_0(0.0, 0.0, 0.0);
    Vector x_axis_0(1.0, 0.0, 0.0);
    Vector y_axis_0(0.0, 1.0, 0.0);
    Vector z_axis_0(0.0, 0.0, 1.0);

    // System 1: translated and rotated 90 degrees around Z
    Point origin_1(10.0, 20.0, 0.0);
    Vector x_axis_1(0.0, 1.0, 0.0);
    Vector y_axis_1(-1.0, 0.0, 0.0);
    Vector z_axis_1(0.0, 0.0, 1.0);

    // Transform maps points FROM system 1 TO system 0
    Xform xform = Xform::change_basis(origin_1, x_axis_1, y_axis_1, z_axis_1, origin_0, x_axis_0, y_axis_0, z_axis_0);

    // Point at origin_1 should map to origin_0
    Point p(10.0, 20.0, 0.0);
    Point tp = xform.transformed_point(p);

    MINI_CHECK(TOLERANCE.is_close(tp[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(tp[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(tp[2], 0.0));
}

MINI_TEST("Xform", "plane_to_plane") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "plane.h"

    // Source plane at origin, XY plane
    Point origin_from(0.0, 0.0, 0.0);
    Vector x_from(1.0, 0.0, 0.0);
    Vector y_from(0.0, 1.0, 0.0);
    Plane plane_from(origin_from, x_from, y_from);

    // Target plane translated and rotated
    Point origin_to(10.0, 0.0, 0.0);
    Vector x_to(0.0, 1.0, 0.0);
    Vector y_to(-1.0, 0.0, 0.0);
    Plane plane_to(origin_to, x_to, y_to);

    Xform xform = Xform::plane_to_plane(plane_from, plane_to);

    // Origin of source should map to origin of target
    Point p(0.0, 0.0, 0.0);
    Point tp = xform.transformed_point(p);

    MINI_CHECK(TOLERANCE.is_close(tp[0], 10.0));
    MINI_CHECK(TOLERANCE.is_close(tp[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(tp[2], 0.0));
}

MINI_TEST("Xform", "look_at_rh") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"

    // Camera at (0,0,10) looking at origin
    Point eye(0.0, 0.0, 10.0);
    Point target(0.0, 0.0, 0.0);
    Vector up(0.0, 1.0, 0.0);

    Xform xform = Xform::look_at_rh(eye, target, up);

    // The target point should be on the negative Z axis in view space
    Point tp = xform.transformed_point(target);

    MINI_CHECK(TOLERANCE.is_close(tp[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(tp[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(tp[2], -10.0));
}

MINI_TEST("Xform", "json_roundtrip") {
    // uncomment #include "xform.h"

    // Create a non-identity xform
    Xform xform = Xform::translation(1.0, 2.0, 3.0);
    xform.name = "test_xform";

    // json_dump(filename) / json_load(filename) - file-based serialization
    std::string filename = "serialization/test_xform.json";
    xform.json_dump(filename);
    Xform loaded = Xform::json_load(filename);

    MINI_CHECK(loaded.name == "test_xform");
    MINI_CHECK(TOLERANCE.is_close(loaded.m[12], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[13], 2.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[14], 3.0));
}

MINI_TEST("Xform", "protobuf_roundtrip") {
    // uncomment #include "xform.h"

    // Create a non-identity xform
    Xform xform = Xform::translation(1.0, 2.0, 3.0);
    xform.name = "test_xform_proto";

    // protobuf_dump(filename) / protobuf_load(filename) - file-based serialization
    std::string filename = "serialization/test_xform.bin";
    xform.protobuf_dump(filename);
    Xform loaded = Xform::protobuf_load(filename);

    MINI_CHECK(loaded.name == "test_xform_proto");
    MINI_CHECK(TOLERANCE.is_close(loaded.m[12], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[13], 2.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[14], 3.0));
}

} // namespace session_cpp
