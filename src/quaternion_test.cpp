#include "mini_test.h"
#include "quaternion.h"
#include "plane.h"
#include "point.h"
#include "encoders.h"
#include "tolerance.h"
#include <filesystem>
#include <cmath>

namespace session_cpp {
using namespace session_cpp::mini_test;

MINI_TEST("Quaternion", "Constructor") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    // Default constructor (identity)
    Quaternion q0;

    // Constructor with arguments
    Quaternion q = Quaternion::from_scalar_and_vector(2.0, Vector(1.0, 0.0, 0.0));

    // Setters
    q[0] = 5.0;
    q[1] = 0.0;
    q[2] = 1.0;
    q[3] = 0.0;

    // Getters
    double s_val = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];

    // Minimal and Full String Representation
    std::string qstr = q.str();
    std::string qrepr = q.repr();

    // Copy (duplicates everything except guid)
    Quaternion qcopy = q.duplicate();
    Quaternion qother = Quaternion::from_scalar_and_vector(2.0, Vector(1.0, 0.0, 0.0));

    // Copy operators
    Quaternion qrot = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
    Quaternion qmul = qrot * qrot;
    Quaternion qscaled = Quaternion::identity() * 2.0;
    Quaternion a = Quaternion::from_scalar_and_vector(1.0, Vector(0.0, 0.0, 0.0));
    Quaternion b = Quaternion::from_scalar_and_vector(0.0, Vector(0.0, 0.0, 1.0));
    Quaternion qsum = a + b;
    Quaternion qdiff = qrot - qrot;
    Quaternion qneg = -Quaternion::identity();

    MINI_CHECK(q0.name == "my_quaternion");
    MINI_CHECK(!q0.guid().empty());
    MINI_CHECK(TOLERANCE.is_close(q0.scalar, 1.0));
    MINI_CHECK(TOLERANCE.is_close(q0.vector[0], 0.0) && TOLERANCE.is_close(q0.vector[1], 0.0) && TOLERANCE.is_close(q0.vector[2], 0.0));
    MINI_CHECK(q[0] == 5.0 && q[1] == 0.0 && q[2] == 1.0 && q[3] == 0.0);
    MINI_CHECK(s_val == 5.0 && x == 0.0 && y == 1.0 && z == 0.0);
    MINI_CHECK(qstr == "5.000000, 0.000000, 1.000000, 0.000000");
    MINI_CHECK(qrepr == "Quaternion(my_quaternion, 5.000000, 0.000000, 1.000000, 0.000000)");
    MINI_CHECK(qcopy == q && qcopy.guid() != q.guid());
    MINI_CHECK(qother != q);
    MINI_CHECK(TOLERANCE.is_close(qmul.scalar, 0.0) && TOLERANCE.is_close(qmul.vector[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(qscaled.scalar, 2.0));
    MINI_CHECK(TOLERANCE.is_close(qsum.scalar, 1.0) && TOLERANCE.is_close(qsum.vector[2], 1.0));
    MINI_CHECK(TOLERANCE.is_close(qdiff.scalar, 0.0) && TOLERANCE.is_close(qdiff.vector[2], 0.0));
    MINI_CHECK(TOLERANCE.is_close(qneg.scalar, -1.0));
}

MINI_TEST("Quaternion", "Identity") {
    // uncomment #include "quaternion.h"

    Quaternion q = Quaternion::identity();

    MINI_CHECK(TOLERANCE.is_close(q.scalar, 1.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[2], 0.0));
}

MINI_TEST("Quaternion", "From Scalar And Vector") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q = Quaternion::from_scalar_and_vector(2.0, Vector(1.0, 2.0, 3.0));

    MINI_CHECK(TOLERANCE.is_close(q.scalar, 2.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[2], 3.0));
}

MINI_TEST("Quaternion", "From Axis Angle") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);

    MINI_CHECK(TOLERANCE.is_close(q.scalar, std::cos(Tolerance::PI / 4.0)));
    MINI_CHECK(TOLERANCE.is_close(q.vector[2], std::sin(Tolerance::PI / 4.0)));
}

MINI_TEST("Quaternion", "From Arc") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

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
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q_euler = Quaternion::from_euler(0.0, 0.0, Tolerance::PI / 2.0);
    Quaternion q_axis = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);

    MINI_CHECK(TOLERANCE.is_close(q_euler.scalar, q_axis.scalar));
    MINI_CHECK(TOLERANCE.is_close(q_euler.vector[2], q_axis.vector[2]));
}

MINI_TEST("Quaternion", "From Rotation") {
    // uncomment #include "quaternion.h"
    // uncomment #include "plane.h"

    Plane plane_a = Plane::xy_plane();
    Plane plane_b(Point(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), Vector(-1.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0));
    Quaternion q = Quaternion::from_rotation(plane_a, plane_b);
    Vector rotated_x = q.rotate_vector(plane_a.x_axis());

    MINI_CHECK(TOLERANCE.is_close(rotated_x[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(rotated_x[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(rotated_x[2], 0.0));
}

MINI_TEST("Quaternion", "Rotate Vector") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
    Vector rotated = q.rotate_vector(Vector(1.0, 0.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(rotated[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(rotated[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(rotated[2], 0.0));
}

MINI_TEST("Quaternion", "Get Rotation") {
    // uncomment #include "quaternion.h"
    // uncomment #include "plane.h"

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
    Plane p = q.get_rotation();

    MINI_CHECK(TOLERANCE.is_close(p.x_axis()[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(p.x_axis()[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(p.y_axis()[0], -1.0));
    MINI_CHECK(TOLERANCE.is_close(p.y_axis()[1], 0.0));
}

MINI_TEST("Quaternion", "Magnitude") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);

    MINI_CHECK(TOLERANCE.is_close(q.magnitude(), 1.0));
}

MINI_TEST("Quaternion", "Magnitude Squared") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);

    MINI_CHECK(TOLERANCE.is_close(q.magnitude_squared(), q.magnitude() * q.magnitude()));
}

MINI_TEST("Quaternion", "Normalized") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q = Quaternion::from_scalar_and_vector(2.0, Vector(0.0, 0.0, 2.0));
    Quaternion n = q.normalized();

    MINI_CHECK(TOLERANCE.is_close(n.magnitude(), 1.0));
}

MINI_TEST("Quaternion", "Conjugate") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);
    Quaternion r = q.conjugate();

    MINI_CHECK(TOLERANCE.is_close(r.scalar, q.scalar));
    MINI_CHECK(TOLERANCE.is_close(r.vector[0], -q.vector[0]));
    MINI_CHECK(TOLERANCE.is_close(r.vector[2], -q.vector[2]));
}

MINI_TEST("Quaternion", "Invert") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 3.0);
    Quaternion result = q * q.invert();

    MINI_CHECK(TOLERANCE.is_close(result.scalar, 1.0));
    MINI_CHECK(TOLERANCE.is_close(result.vector[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(result.vector[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(result.vector[2], 0.0));
}

MINI_TEST("Quaternion", "Dot") {
    // uncomment #include "quaternion.h"

    Quaternion q = Quaternion::identity();

    MINI_CHECK(TOLERANCE.is_close(q.dot(q), 1.0));
}

MINI_TEST("Quaternion", "Slerp") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q1 = Quaternion::identity();
    Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
    Quaternion mid = q1.slerp(q2, 0.5);
    Quaternion expected = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);

    MINI_CHECK(TOLERANCE.is_close(mid.scalar, expected.scalar));
    MINI_CHECK(TOLERANCE.is_close(mid.vector[2], expected.vector[2]));
    Quaternion q3 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), 0.001);
    Quaternion mid2 = q1.slerp(q3, 0.5);
    Quaternion half = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), 0.0005);
    MINI_CHECK(TOLERANCE.is_close(mid2.scalar, half.scalar));
}

MINI_TEST("Quaternion", "Nlerp") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    Quaternion q1 = Quaternion::identity();
    Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
    Quaternion r0 = q1.nlerp(q2, 0.0);
    Quaternion r1 = q1.nlerp(q2, 1.0);

    MINI_CHECK(TOLERANCE.is_close(r0.scalar, q1.scalar));
    MINI_CHECK(TOLERANCE.is_close(r1.scalar, q2.scalar));
}

MINI_TEST("Quaternion", "Json Roundtrip") {
    // uncomment #include "quaternion.h"

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
    q.name = "serialization/test_quaternion";

    //   jsondump()      │ ordered_json │ to JSON object (internal use)
    //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
    //   json_dumps()    │ std::string  │ to JSON string
    //   json_loads(s)   │ std::string  │ from JSON string
    //   json_dump(path) │ file         │ write to file
    //   json_load(path) │ file         │ read from file

    std::string filename = "serialization/test_quaternion.json";
    q.json_dump(filename);
    Quaternion loaded = Quaternion::json_load(filename);

    MINI_CHECK(loaded.name == "serialization/test_quaternion");
    MINI_CHECK(TOLERANCE.is_close(loaded.scalar, q.scalar));
    MINI_CHECK(TOLERANCE.is_close(loaded.vector[2], q.vector[2]));
}

MINI_TEST("Quaternion", "Protobuf Roundtrip") {
    // uncomment #include "quaternion.h"

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
    q.name = "serialization/test_quaternion";

    std::string filename = "serialization/test_quaternion.bin";
    q.pb_dump(filename);
    Quaternion loaded = Quaternion::pb_load(filename);

    MINI_CHECK(loaded.name == "serialization/test_quaternion");
    MINI_CHECK(TOLERANCE.is_close(loaded.scalar, q.scalar));
    MINI_CHECK(TOLERANCE.is_close(loaded.vector[2], q.vector[2]));
}

} // namespace session_cpp
