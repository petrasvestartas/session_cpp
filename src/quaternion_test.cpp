#include "mini_test.h"
#include "quaternion.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>
#include <cmath>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Quaternion", "Json Roundtrip") {
    Vector axis(0.0, 0.0, 1.0);
    Quaternion original = Quaternion::from_axis_angle(axis, 1.5708);
    std::filesystem::create_directories("./serialization");
    encoders::json_dump(original, "./serialization/test_quaternion.json");
    Quaternion loaded = encoders::json_load<Quaternion>("./serialization/test_quaternion.json");

    MINI_CHECK(TOLERANCE.is_close(loaded.s, original.s));
}

MINI_TEST("Quaternion", "From Arc") {
    Vector src(1.0, 0.0, 0.0);
    Vector dst(0.0, 1.0, 0.0);
    Quaternion q = Quaternion::from_arc(src, dst);
    Vector rotated = q.rotate_vector(src);

    MINI_CHECK(TOLERANCE.is_close(rotated[0], dst[0]));
    MINI_CHECK(TOLERANCE.is_close(rotated[1], dst[1]));
    MINI_CHECK(TOLERANCE.is_close(rotated[2], dst[2]));
    Vector src2(1.0, 0.0, 0.0);
    Vector dst2(-1.0, 0.0, 0.0);
    Quaternion q2 = Quaternion::from_arc(src2, dst2);
    Vector rot = q2.rotate_vector(src2);
    MINI_CHECK(TOLERANCE.is_close(rot[0], -1.0));
    MINI_CHECK(TOLERANCE.is_close(rot[1], 0.0));
}

MINI_TEST("Quaternion", "From Euler") {
    double pi = std::acos(-1.0);
    Quaternion q_euler = Quaternion::from_euler(0.0, 0.0, pi / 2.0);
    Quaternion q_axis = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), pi / 2.0);

    MINI_CHECK(TOLERANCE.is_close(q_euler.s, q_axis.s));
    MINI_CHECK(TOLERANCE.is_close(q_euler.v[2], q_axis.v[2]));
}

MINI_TEST("Quaternion", "slerp") {
    double pi = std::acos(-1.0);
    Quaternion q1 = Quaternion::identity();
    Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), pi / 2.0);
    Quaternion mid = q1.slerp(q2, 0.5);
    Quaternion expected = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), pi / 4.0);

    MINI_CHECK(TOLERANCE.is_close(mid.s, expected.s));
    MINI_CHECK(TOLERANCE.is_close(mid.v[2], expected.v[2]));
    Quaternion q3 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), 0.001);
    Quaternion mid2 = q1.slerp(q3, 0.5);
    Quaternion half = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), 0.0005);
    MINI_CHECK(TOLERANCE.is_close(mid2.s, half.s));
}

MINI_TEST("Quaternion", "nlerp") {
    double pi = std::acos(-1.0);
    Quaternion q1 = Quaternion::identity();
    Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), pi / 2.0);
    Quaternion r0 = q1.nlerp(q2, 0.0);
    Quaternion r1 = q1.nlerp(q2, 1.0);

    MINI_CHECK(TOLERANCE.is_close(r0.s, q1.s));
    MINI_CHECK(TOLERANCE.is_close(r1.s, q2.s));
}

MINI_TEST("Quaternion", "invert") {
    double pi = std::acos(-1.0);
    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), pi / 3.0);
    Quaternion result = q * q.invert();

    MINI_CHECK(TOLERANCE.is_close(result.s, 1.0));
    MINI_CHECK(TOLERANCE.is_close(result.v[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(result.v[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(result.v[2], 0.0));
}

MINI_TEST("Quaternion", "dot") {
    Quaternion q = Quaternion::identity();

    MINI_CHECK(TOLERANCE.is_close(q.dot(q), 1.0));
}

MINI_TEST("Quaternion", "magnitude2") {
    double pi = std::acos(-1.0);
    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), pi / 4.0);

    MINI_CHECK(TOLERANCE.is_close(q.magnitude2(), q.magnitude() * q.magnitude()));
}

MINI_TEST("Quaternion", "add") {
    Quaternion a = Quaternion::from_sv(1.0, Vector(0.0, 0.0, 0.0));
    Quaternion b = Quaternion::from_sv(0.0, Vector(0.0, 0.0, 1.0));
    Quaternion r = a + b;

    MINI_CHECK(TOLERANCE.is_close(r.s, 1.0));
    MINI_CHECK(TOLERANCE.is_close(r.v[2], 1.0));
}

MINI_TEST("Quaternion", "sub") {
    double pi = std::acos(-1.0);
    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), pi / 4.0);
    Quaternion r = q - q;

    MINI_CHECK(TOLERANCE.is_close(r.s, 0.0));
    MINI_CHECK(TOLERANCE.is_close(r.v[2], 0.0));
}

MINI_TEST("Quaternion", "neg") {
    Quaternion q = Quaternion::identity();
    Quaternion r = -q;

    MINI_CHECK(TOLERANCE.is_close(r.s, -1.0));
}

MINI_TEST("Quaternion", "Mul Scalar") {
    Quaternion q = Quaternion::identity();
    Quaternion r = q * 2.0;

    MINI_CHECK(TOLERANCE.is_close(r.s, 2.0));
}

MINI_TEST("Quaternion", "conjugate") {
    double pi = std::acos(-1.0);
    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), pi / 4.0);
    Quaternion r = q.conjugate();

    MINI_CHECK(TOLERANCE.is_close(r.s, q.s));
    MINI_CHECK(TOLERANCE.is_close(r.v[0], -q.v[0]));
    MINI_CHECK(TOLERANCE.is_close(r.v[2], -q.v[2]));
}

MINI_TEST("Quaternion", "mul") {
    double pi = std::acos(-1.0);
    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), pi / 2.0);
    Quaternion r = q * q;
    Vector v = r.rotate_vector(Vector(1.0, 0.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(v[0], -1.0));
    MINI_CHECK(TOLERANCE.is_close(v[1], 0.0));
}

} // namespace session_cpp
