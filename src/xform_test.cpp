#include "mini_test.h"
#include "xform.h"
#include "point.h"
#include "vector.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Xform", "constructor") {
    // uncomment #include "xform.h"

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

    MINI_CHECK(x.name == "my_xform" && !x.guid.empty());
    MINI_CHECK(m00 == 1.0 && m11 == 1.0 && m22 == 1.0 && m33 == 1.0);
    MINI_CHECK(is_id == true);
    MINI_CHECK(xfrom.m[12] == 5.0 && xfrom.m[13] == 10.0 && xfrom.m[14] == 15.0);
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

MINI_TEST("Xform", "rotation_z") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "tolerance.h"

    // Rotation around Z axis by 90 degrees
    Xform r = Xform::rotation_z(Tolerance::PI / 2.0);

    // Apply to point (1,0,0) -> (0,1,0)
    Point p(1.0, 0.0, 0.0);
    Point rp = r.transformed_point(p);

    MINI_CHECK(TOLERANCE.is_close(rp[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(rp[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(rp[2], 0.0));
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

MINI_TEST("Xform", "mul_operator") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"

    // Matrix multiplication
    Xform t = Xform::translation(10.0, 0.0, 0.0);
    Xform s = Xform::scaling(2.0, 1.0, 1.0);

    // Combined: first scale, then translate
    Xform combined = t * s;

    // Apply to point
    Point p(1.0, 0.0, 0.0);
    Point result = combined.transformed_point(p);

    // (1,0,0) * scale(2,1,1) = (2,0,0), then translate(10,0,0) = (12,0,0)
    MINI_CHECK(TOLERANCE.is_close(result[0], 12.0));
    MINI_CHECK(TOLERANCE.is_close(result[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(result[2], 0.0));
}

MINI_TEST("Xform", "transform_vector") {
    // uncomment #include "xform.h"
    // uncomment #include "vector.h"

    // Translation should not affect vectors (only direction)
    Xform t = Xform::translation(100.0, 200.0, 300.0);
    Vector v(1.0, 0.0, 0.0);
    Vector tv = t.transformed_vector(v);

    // Scaling should affect vectors
    Xform s = Xform::scaling(2.0, 3.0, 4.0);
    Vector v2(1.0, 1.0, 1.0);
    Vector sv = s.transformed_vector(v2);

    MINI_CHECK(TOLERANCE.is_close(tv[0], 1.0) && TOLERANCE.is_close(tv[1], 0.0) && TOLERANCE.is_close(tv[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(sv[0], 2.0) && TOLERANCE.is_close(sv[1], 3.0) && TOLERANCE.is_close(sv[2], 4.0));
}

MINI_TEST("Xform", "rotation_x") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "tolerance.h"

    // Rotation around X axis by 90 degrees
    Xform r = Xform::rotation_x(Tolerance::PI / 2.0);

    // Apply to point (0,1,0) -> (0,0,1)
    Point p(0.0, 1.0, 0.0);
    Point rp = r.transformed_point(p);

    MINI_CHECK(TOLERANCE.is_close(rp[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(rp[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(rp[2], 1.0));
}

MINI_TEST("Xform", "rotation_y") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "tolerance.h"

    // Rotation around Y axis by 90 degrees
    Xform r = Xform::rotation_y(Tolerance::PI / 2.0);

    // Apply to point (0,0,1) -> (1,0,0)
    Point p(0.0, 0.0, 1.0);
    Point rp = r.transformed_point(p);

    MINI_CHECK(TOLERANCE.is_close(rp[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(rp[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(rp[2], 0.0));
}

MINI_TEST("Xform", "rotation") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "tolerance.h"

    // Rotation around arbitrary axis (1,1,1) by 120 degrees
    // This cycles x->y->z->x
    Vector axis(1.0, 1.0, 1.0);
    Xform r = Xform::rotation(axis, 2.0 * Tolerance::PI / 3.0);

    // Apply to point (1,0,0) -> (0,1,0)
    Point p(1.0, 0.0, 0.0);
    Point rp = r.transformed_point(p);

    MINI_CHECK(TOLERANCE.is_close(rp[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(rp[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(rp[2], 0.0));
}

MINI_TEST("Xform", "change_basis") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"

    // Create a coordinate system at origin with rotated axes
    Point origin(10.0, 20.0, 30.0);
    Vector x_axis(1.0, 0.0, 0.0);
    Vector y_axis(0.0, 1.0, 0.0);
    Vector z_axis(0.0, 0.0, 1.0);

    // Change basis transform
    Xform xform = Xform::change_basis(origin, x_axis, y_axis, z_axis);

    // Point at local origin should map to world origin
    Point p(10.0, 20.0, 30.0);
    Point tp = xform.transformed_point(p);

    MINI_CHECK(TOLERANCE.is_close(tp[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(tp[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(tp[2], 0.0));
}

MINI_TEST("Xform", "plane_to_plane") {
    // uncomment #include "xform.h"
    // uncomment #include "point.h"
    // uncomment #include "vector.h"

    // Source plane at origin, XY plane
    Point origin_0(0.0, 0.0, 0.0);
    Vector x_axis_0(1.0, 0.0, 0.0);
    Vector y_axis_0(0.0, 1.0, 0.0);
    Vector z_axis_0(0.0, 0.0, 1.0);

    // Target plane translated and rotated
    Point origin_1(10.0, 0.0, 0.0);
    Vector x_axis_1(0.0, 1.0, 0.0);
    Vector y_axis_1(-1.0, 0.0, 0.0);
    Vector z_axis_1(0.0, 0.0, 1.0);

    Xform xform = Xform::plane_to_plane(origin_0, x_axis_0, y_axis_0, z_axis_0, origin_1, x_axis_1, y_axis_1, z_axis_1);

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
    std::string filename = "test_xform.json";
    xform.json_dump(filename);
    Xform loaded = Xform::json_load(filename);

    MINI_CHECK(loaded.name == "test_xform");
    MINI_CHECK(TOLERANCE.is_close(loaded.m[12], 1.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[13], 2.0));
    MINI_CHECK(TOLERANCE.is_close(loaded.m[14], 3.0));
}

} // namespace session_cpp
