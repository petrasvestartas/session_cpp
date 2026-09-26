#include "mini_test.h"
#include "quaternion.h"
#include "quaternion.pb.h"
#include "plane.h"
#include "point.h"
#include "tolerance.h"
#include <cmath>
#include <string>
#include <utility>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Quaternion", "Constructor") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q0;
        Quaternion q = Quaternion::from_components(2.0, Vector(1.0, 0.0, 0.0));

        q[0] = 5.0;
        q[1] = 0.0;
        q[2] = 1.0;
        q[3] = 0.0;

        const double s_val = q[0];
        const double x = q[1];
        const double y = q[2];
        const double z = q[3];

        const std::string qstr = q.str();
        const std::string qrepr = q.repr();

        const Quaternion qcopy = q.duplicate();
        const Quaternion qother = Quaternion::from_components(2.0, Vector(1.0, 0.0, 0.0));

        const Quaternion qrot = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
        const Quaternion qmul = qrot * qrot;
        const Quaternion qscaled = Quaternion::identity() * 2.0;
        const Quaternion a = Quaternion::from_components(1.0, Vector(0.0, 0.0, 0.0));
        const Quaternion b = Quaternion::from_components(0.0, Vector(0.0, 0.0, 1.0));
        const Quaternion qsum = a + b;
        const Quaternion qdiff = qrot - qrot;
        const Quaternion qneg = -Quaternion::identity();

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
        // using session_cpp::Quaternion;

        const Quaternion q = Quaternion::identity();

        MINI_CHECK(TOLERANCE.is_close(q.scalar, 1.0));
        MINI_CHECK(TOLERANCE.is_close(q.vector[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(q.vector[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(q.vector[2], 0.0));
    }

    MINI_TEST("Quaternion", "From Components") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q = Quaternion::from_components(2.0, Vector(1.0, 2.0, 3.0));

        MINI_CHECK(TOLERANCE.is_close(q.scalar, 2.0));
        MINI_CHECK(TOLERANCE.is_close(q.vector[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(q.vector[1], 2.0));
        MINI_CHECK(TOLERANCE.is_close(q.vector[2], 3.0));

        const std::pair<Vector, double> axis_angle = q.to_axis_angle();
        const Vector axis = axis_angle.first;
        const double angle = axis_angle.second;
        const double sqrt14 = std::sqrt(14.0);

        MINI_CHECK(TOLERANCE.is_close(axis[0], 1.0 / sqrt14));
        MINI_CHECK(TOLERANCE.is_close(axis[1], 2.0 / sqrt14));
        MINI_CHECK(TOLERANCE.is_close(axis[2], 3.0 / sqrt14));
        MINI_CHECK(TOLERANCE.is_close(angle, 2.0 * std::acos(2.0 / std::sqrt(18.0))));

        const Quaternion q_round = Quaternion::from_axis_angle(axis, angle);
        const Quaternion qn = q.normalized();

        MINI_CHECK(TOLERANCE.is_close(q_round.scalar, qn.scalar));
        MINI_CHECK(TOLERANCE.is_close(q_round.vector[0], qn.vector[0]));
        MINI_CHECK(TOLERANCE.is_close(q_round.vector[1], qn.vector[1]));
        MINI_CHECK(TOLERANCE.is_close(q_round.vector[2], qn.vector[2]));
    }

    MINI_TEST("Quaternion", "From Axis Angle") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);

        MINI_CHECK(TOLERANCE.is_close(q.scalar, std::cos(Tolerance::PI / 4.0)));
        MINI_CHECK(TOLERANCE.is_close(q.vector[2], std::sin(Tolerance::PI / 4.0)));

        const Quaternion zero_axis = Quaternion::from_axis_angle(Vector(0.0, 0.0, 0.0), Tolerance::PI / 2.0);

        MINI_CHECK(zero_axis == Quaternion::identity());
    }

    MINI_TEST("Quaternion", "From Arc") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Vector src(1.0, 0.0, 0.0);
        const Vector dst(0.0, 1.0, 0.0);
        const Quaternion q = Quaternion::from_arc(src, dst);
        const Vector rotated = q.rotate_vector(src);

        MINI_CHECK(TOLERANCE.is_close(rotated[0], dst[0]));
        MINI_CHECK(TOLERANCE.is_close(rotated[1], dst[1]));
        MINI_CHECK(TOLERANCE.is_close(rotated[2], dst[2]));

        const Vector src2(1.0, 0.0, 0.0);
        const Vector dst2(-1.0, 0.0, 0.0);
        const Quaternion q2 = Quaternion::from_arc(src2, dst2);
        const Vector rot = q2.rotate_vector(src2);

        MINI_CHECK(TOLERANCE.is_close(rot[0], -1.0));
        MINI_CHECK(TOLERANCE.is_close(rot[1], 0.0));
    }

    MINI_TEST("Quaternion", "From Euler") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q_euler = Quaternion::from_euler(0.0, 0.0, Tolerance::PI / 2.0);
        const Quaternion q_axis = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);

        MINI_CHECK(TOLERANCE.is_close(q_euler.scalar, q_axis.scalar));
        MINI_CHECK(TOLERANCE.is_close(q_euler.vector[2], q_axis.vector[2]));
    }

    MINI_TEST("Quaternion", "From Rotation") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;
        // using session_cpp::Plane;
        // using session_cpp::Point;

        const Plane plane_a = Plane::xy_plane();
        const Plane plane_b = Plane::from_frame(
            Point(0.0, 0.0, 0.0),
            Vector(0.0, 1.0, 0.0),
            Vector(-1.0, 0.0, 0.0),
            Vector(0.0, 0.0, 1.0)
        );
        const Quaternion q = Quaternion::from_rotation(plane_a, plane_b);
        const Vector rotated_x = q.rotate_vector(plane_a.x_axis());

        MINI_CHECK(TOLERANCE.is_close(rotated_x[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(rotated_x[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(rotated_x[2], 0.0));
    }

    MINI_TEST("Quaternion", "Rotate Vector") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
        const Vector rotated = q.rotate_vector(Vector(1.0, 0.0, 0.0));

        MINI_CHECK(TOLERANCE.is_close(rotated[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(rotated[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(rotated[2], 0.0));
    }

    MINI_TEST("Quaternion", "Get Rotation") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;
        // using session_cpp::Plane;

        const Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
        const Plane p = q.get_rotation();

        MINI_CHECK(TOLERANCE.is_close(p.x_axis()[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(p.x_axis()[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(p.y_axis()[0], -1.0));
        MINI_CHECK(TOLERANCE.is_close(p.y_axis()[1], 0.0));
    }

    MINI_TEST("Quaternion", "Magnitude") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);

        MINI_CHECK(TOLERANCE.is_close(q.magnitude(), 1.0));
    }

    MINI_TEST("Quaternion", "Magnitude Squared") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);

        MINI_CHECK(TOLERANCE.is_close(q.magnitude_squared(), q.magnitude() * q.magnitude()));
    }

    MINI_TEST("Quaternion", "Normalized") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q = Quaternion::from_components(2.0, Vector(0.0, 0.0, 2.0));
        const Quaternion n = q.normalized();

        MINI_CHECK(TOLERANCE.is_close(n.magnitude(), 1.0));
    }

    MINI_TEST("Quaternion", "Conjugate") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);
        const Quaternion r = q.conjugate();

        MINI_CHECK(TOLERANCE.is_close(r.scalar, q.scalar));
        MINI_CHECK(TOLERANCE.is_close(r.vector[0], -q.vector[0]));
        MINI_CHECK(TOLERANCE.is_close(r.vector[2], -q.vector[2]));
    }

    MINI_TEST("Quaternion", "Invert") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 3.0);
        const Quaternion result = q * q.invert();

        MINI_CHECK(TOLERANCE.is_close(result.scalar, 1.0));
        MINI_CHECK(TOLERANCE.is_close(result.vector[0], 0.0));
        MINI_CHECK(TOLERANCE.is_close(result.vector[1], 0.0));
        MINI_CHECK(TOLERANCE.is_close(result.vector[2], 0.0));
    }

    MINI_TEST("Quaternion", "Dot") {
        // using session_cpp::Quaternion;

        const Quaternion q = Quaternion::identity();

        MINI_CHECK(TOLERANCE.is_close(q.dot(q), 1.0));
    }

    MINI_TEST("Quaternion", "Slerp") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q1 = Quaternion::identity();
        const Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
        const Quaternion mid = q1.slerp(q2, 0.5);
        const Quaternion expected = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);

        MINI_CHECK(TOLERANCE.is_close(mid.scalar, expected.scalar));
        MINI_CHECK(TOLERANCE.is_close(mid.vector[2], expected.vector[2]));

        const Quaternion q3 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), 0.001);
        const Quaternion mid2 = q1.slerp(q3, 0.5);
        const Quaternion half = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), 0.0005);

        MINI_CHECK(TOLERANCE.is_close(mid2.scalar, half.scalar));

        const Quaternion antipodal = -Quaternion::identity();
        const Quaternion same_rotation = q1.slerp(antipodal, 0.5);

        MINI_CHECK(TOLERANCE.is_close(same_rotation.scalar, 1.0));
        MINI_CHECK(TOLERANCE.is_close(same_rotation.vector.magnitude(), 0.0));
    }

    MINI_TEST("Quaternion", "Nlerp") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        const Quaternion q1 = Quaternion::identity();
        const Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
        const Quaternion r0 = q1.nlerp(q2, 0.0);
        const Quaternion r1 = q1.nlerp(q2, 1.0);

        MINI_CHECK(TOLERANCE.is_close(r0.scalar, q1.scalar));
        MINI_CHECK(TOLERANCE.is_close(r1.scalar, q2.scalar));
    }

    MINI_TEST("Quaternion", "Json Roundtrip") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
        q.name = "test_quaternion";

        const std::string guid = q.guid();
        const std::string filename = "serialization/test_quaternion.json";
        q.file_json_dump(filename);

        const Quaternion loaded = Quaternion::file_json_load(filename);
        const Quaternion parsed = Quaternion::file_json_loads(q.file_json_dumps());

        MINI_CHECK(loaded.name == "test_quaternion");
        MINI_CHECK(TOLERANCE.is_close(loaded.scalar, q.scalar));
        MINI_CHECK(TOLERANCE.is_close(loaded.vector[2], q.vector[2]));
        MINI_CHECK(parsed == q);
        MINI_CHECK(loaded.guid() == guid);
        MINI_CHECK(parsed.guid() == guid);
    }

    MINI_TEST("Quaternion", "Protobuf Roundtrip") {
        // using session_cpp::Quaternion;
        // using session_cpp::Vector;

        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
        q.name = "test_quaternion";

        const std::string filename = "serialization/test_quaternion.bin";
        q.pb_dump(filename);

        const Quaternion loaded = Quaternion::pb_load(filename);
        const Quaternion parsed = Quaternion::pb_loads(q.pb_dumps());
        const Quaternion converted = Quaternion::from_proto(q.to_proto());

        MINI_CHECK(loaded.name == "test_quaternion");
        MINI_CHECK(TOLERANCE.is_close(loaded.scalar, q.scalar));
        MINI_CHECK(TOLERANCE.is_close(loaded.vector[2], q.vector[2]));
        MINI_CHECK(parsed == q);
        MINI_CHECK(converted == q);
    }

} // namespace session_cpp
