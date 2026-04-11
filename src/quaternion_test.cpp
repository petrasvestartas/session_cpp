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

    // FUNCTION: Quaternion constructors and operators
    // WHAT: Create quaternions and access components, copy them, and combine
    //       them with arithmetic. Multiplication composes rotations:
    //       q3 = q2 * q1 means "apply q1 first, then q2".
    // WHEN TO USE: You will rarely call the raw constructor directly. The
    //       factories (from_axis_angle, from_arc, from_euler) are what you
    //       reach for in practice. The * operator is what chains rotations.
    // TEST: Exercises default ctor, indexing, duplicate, equality, +, -, *, neg.

    // Default constructor (identity)
    Quaternion q0;

    // Constructor with arguments
    Quaternion q = Quaternion::from_components(2.0, Vector(1.0, 0.0, 0.0));

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
    Quaternion qother = Quaternion::from_components(2.0, Vector(1.0, 0.0, 0.0));

    // Copy operators
    Quaternion qrot = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
    Quaternion qmul = qrot * qrot;
    Quaternion qscaled = Quaternion::identity() * 2.0;
    Quaternion a = Quaternion::from_components(1.0, Vector(0.0, 0.0, 0.0));
    Quaternion b = Quaternion::from_components(0.0, Vector(0.0, 0.0, 1.0));
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

    // FUNCTION: Quaternion::identity()
    // WHAT: The "do nothing" rotation. Rotating any vector by identity returns
    //       it unchanged. Equivalent to (scalar=1, vector=(0,0,0)).
    // WHEN TO USE: As the starting orientation of an object that has not been
    //       rotated yet, or as the start point of an interpolation like slerp.

    Quaternion q = Quaternion::identity();

    MINI_CHECK(TOLERANCE.is_close(q.scalar, 1.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[2], 0.0));
}

MINI_TEST("Quaternion", "From Components") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    // q = s + xi + yj + zk: first arg is scalar, second arg is (i,j,k) coefficients (NOT a rotation axis).
    Quaternion q = Quaternion::from_components(2.0, Vector(1.0, 2.0, 3.0));

    MINI_CHECK(TOLERANCE.is_close(q.scalar, 2.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[0], 1.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[1], 2.0));
    MINI_CHECK(TOLERANCE.is_close(q.vector[2], 3.0));

    // Geometric meaning of (s, v): a rotation by `angle` around `axis`.
    // to_axis_angle() extracts these — for q=(2,(1,2,3)):
    //   axis  = (1,2,3)/sqrt(14)
    //   angle = 2*acos(2/sqrt(18)) ≈ 2.1617 rad ≈ 123.85°
    auto axis_angle = q.to_axis_angle();
    Vector axis = axis_angle.first;
    double angle = axis_angle.second;
    double sqrt14 = std::sqrt(14.0);
    MINI_CHECK(TOLERANCE.is_close(axis[0], 1.0 / sqrt14));
    MINI_CHECK(TOLERANCE.is_close(axis[1], 2.0 / sqrt14));
    MINI_CHECK(TOLERANCE.is_close(axis[2], 3.0 / sqrt14));
    MINI_CHECK(TOLERANCE.is_close(angle, 2.0 * std::acos(2.0 / std::sqrt(18.0))));

    // Round-trip: from_axis_angle(to_axis_angle(q)) == q.normalized()
    Quaternion q_round = Quaternion::from_axis_angle(axis, angle);
    Quaternion qn = q.normalized();
    MINI_CHECK(TOLERANCE.is_close(q_round.scalar, qn.scalar));
    MINI_CHECK(TOLERANCE.is_close(q_round.vector[0], qn.vector[0]));
    MINI_CHECK(TOLERANCE.is_close(q_round.vector[1], qn.vector[1]));
    MINI_CHECK(TOLERANCE.is_close(q_round.vector[2], qn.vector[2]));
}

MINI_TEST("Quaternion", "From Axis Angle") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    // FUNCTION: Quaternion::from_axis_angle(axis, angle)
    // WHAT: Build a unit quaternion that rotates by `angle` radians around `axis`.
    //       This is the geometric meaning most people expect.
    // WHEN TO USE: The everyday rotation builder. Anytime you can describe the
    //       rotation as "spin by N radians around this direction" - turning a
    //       wheel, opening a door, orbiting a camera - this is the constructor.
    // TEST: 90 deg around Z. scalar=cos(45 deg), z=sin(45 deg).

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);

    MINI_CHECK(TOLERANCE.is_close(q.scalar, std::cos(Tolerance::PI / 4.0)));
    MINI_CHECK(TOLERANCE.is_close(q.vector[2], std::sin(Tolerance::PI / 4.0)));
}

MINI_TEST("Quaternion", "From Arc") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    // FUNCTION: Quaternion::from_arc(src, dst)
    // WHAT: Build the shortest rotation that maps direction `src` to direction
    //       `dst`. The rotation axis is perpendicular to both vectors.
    // WHEN TO USE: Look-at logic (point a camera at a target), aligning a
    //       model's forward axis with a desired direction, snapping one face
    //       normal to another.
    // TEST: Rotate (1,0,0) to (0,1,0); also handles the 180 deg flip case.

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

    // FUNCTION: Quaternion::from_euler(x, y, z)
    // WHAT: Build a quaternion from three Euler angles (XYZ convention).
    // WHEN TO USE: Only at I/O boundaries (importing pitch/yaw/roll, UI input).
    // AVOID WHEN: Composing many rotations - Euler angles suffer from gimbal
    //       lock. Store/compose as quaternions, convert to Euler only to save.
    // TEST: Euler (0, 0, PI/2) equals from_axis_angle(Z, PI/2).

    Quaternion q_euler = Quaternion::from_euler(0.0, 0.0, Tolerance::PI / 2.0);
    Quaternion q_axis = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);

    MINI_CHECK(TOLERANCE.is_close(q_euler.scalar, q_axis.scalar));
    MINI_CHECK(TOLERANCE.is_close(q_euler.vector[2], q_axis.vector[2]));
}

MINI_TEST("Quaternion", "From Rotation") {
    // uncomment #include "quaternion.h"
    // uncomment #include "plane.h"

    // FUNCTION: Quaternion::from_rotation(plane_a, plane_b)
    // WHAT: Build the quaternion that maps the basis of plane_a onto the basis
    //       of plane_b. Equivalent to "rotate a's frame to match b's frame".
    // WHEN TO USE: CAD assembly - aligning two parts by matching their
    //       reference planes; transferring a local frame between objects;
    //       expressing the relative rotation between two coordinate systems.
    // TEST: Maps the world XY plane to a 90-deg-rotated frame; checks x_axis.

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

    // FUNCTION: q.rotate_vector(v)
    // WHAT: Apply this rotation to a 3D vector and return the rotated vector.
    //       Mathematically: q * v_pure * q.conjugate(), where v_pure = (0, v).
    // WHEN TO USE: You have a quaternion orientation and need to know where a
    //       specific direction points after the rotation - the camera's
    //       forward axis, a bone's tip, the normal of a rotated face.
    // TEST: Rotating X by 90 deg around Z gives Y.

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
    Vector rotated = q.rotate_vector(Vector(1.0, 0.0, 0.0));

    MINI_CHECK(TOLERANCE.is_close(rotated[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(rotated[1], 1.0));
    MINI_CHECK(TOLERANCE.is_close(rotated[2], 0.0));
}

MINI_TEST("Quaternion", "Get Rotation") {
    // uncomment #include "quaternion.h"
    // uncomment #include "plane.h"

    // FUNCTION: q.get_rotation()
    // WHAT: Apply this rotation to the world XY plane and return the resulting
    //       Plane. Equivalent to a Plane whose axes are the rotated X, Y, Z.
    // WHEN TO USE: Visualize a quaternion as a frame, or convert a stored
    //       quaternion into a Plane for frame-based APIs in the rest of the
    //       kernel. Inverse of from_rotation(xy_plane(), result).
    // TEST: 90 deg around Z gives a frame whose x_axis = Y, y_axis = -X.

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

    // FUNCTION: q.magnitude()
    // WHAT: 4D length sqrt(s^2 + x^2 + y^2 + z^2). Always 1 for a valid
    //       rotation quaternion (unit quaternion).
    // WHEN TO USE: Debug assertions ("is this still unit?"), or before calling
    //       normalized() to decide whether normalization is needed.

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);

    MINI_CHECK(TOLERANCE.is_close(q.magnitude(), 1.0));
}

MINI_TEST("Quaternion", "Magnitude Squared") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    // FUNCTION: q.magnitude_squared()
    // WHAT: Same as magnitude() but skips the sqrt. Cheaper.
    // WHEN TO USE: Comparing relative magnitudes, or in distance-based
    //       similarity checks where the absolute value isn't needed.

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);

    MINI_CHECK(TOLERANCE.is_close(q.magnitude_squared(), q.magnitude() * q.magnitude()));
}

MINI_TEST("Quaternion", "Normalized") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    // FUNCTION: q.normalized()
    // WHAT: Return a unit-length copy of this quaternion (divides by magnitude).
    // WHEN TO USE: Periodically after composing many rotations in a loop -
    //       floating-point drift slowly makes a quaternion non-unit, and a
    //       non-unit quaternion no longer represents a valid rotation.

    Quaternion q = Quaternion::from_components(2.0, Vector(0.0, 0.0, 2.0));
    Quaternion n = q.normalized();

    MINI_CHECK(TOLERANCE.is_close(n.magnitude(), 1.0));
}

MINI_TEST("Quaternion", "Conjugate") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    // FUNCTION: q.conjugate()
    // WHAT: Flip the sign of the vector part: (s, v) -> (s, -v). For UNIT
    //       quaternions this equals the inverse - the opposite rotation.
    // WHEN TO USE: When you need the inverse rotation AND you know the
    //       quaternion is unit-length. Cheaper than invert() (no division).

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 4.0);
    Quaternion r = q.conjugate();

    MINI_CHECK(TOLERANCE.is_close(r.scalar, q.scalar));
    MINI_CHECK(TOLERANCE.is_close(r.vector[0], -q.vector[0]));
    MINI_CHECK(TOLERANCE.is_close(r.vector[2], -q.vector[2]));
}

MINI_TEST("Quaternion", "Invert") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    // FUNCTION: q.invert()
    // WHAT: True multiplicative inverse: conjugate / magnitude_squared. Works
    //       for non-unit quaternions too.
    // WHEN TO USE: When you need q^-1 and you are NOT sure the quaternion is
    //       unit. q * q.invert() always equals identity by definition.

    Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 3.0);
    Quaternion result = q * q.invert();

    MINI_CHECK(TOLERANCE.is_close(result.scalar, 1.0));
    MINI_CHECK(TOLERANCE.is_close(result.vector[0], 0.0));
    MINI_CHECK(TOLERANCE.is_close(result.vector[1], 0.0));
    MINI_CHECK(TOLERANCE.is_close(result.vector[2], 0.0));
}

MINI_TEST("Quaternion", "Dot") {
    // uncomment #include "quaternion.h"

    // FUNCTION: q1.dot(q2)
    // WHAT: Algebraic 4D dot product s1*s2 + x1*x2 + y1*y2 + z1*z2. NOT a
    //       geometric operation - just the dot product of the 4 components.
    // WHEN TO USE: Inside slerp implementations, or as a similarity measure
    //       between two unit quaternions (1 = same, 0 = 90 deg apart).

    Quaternion q = Quaternion::identity();

    MINI_CHECK(TOLERANCE.is_close(q.dot(q), 1.0));
}

MINI_TEST("Quaternion", "Slerp") {
    // uncomment #include "quaternion.h"
    // uncomment #include "vector.h"

    // FUNCTION: q1.slerp(q2, t)
    // WHAT: Spherical Linear intERPolation. Smoothly blends q1 -> q2 along the
    //       shortest great-circle path on S^3. Constant angular velocity.
    // WHEN TO USE: High-quality animation between two orientations - camera
    //       transitions, character bones, anything where smoothness matters
    //       more than raw speed.
    // TEST: Midpoint of slerp(identity, 90 deg) is 45 deg.

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

    // FUNCTION: q1.nlerp(q2, t)
    // WHAT: Normalized Linear intERPolation. Cheaper than slerp but the
    //       angular velocity isn't perfectly uniform. Often "good enough".
    // WHEN TO USE: Real-time loops where every microsecond matters and the
    //       visual difference from slerp is negligible (game animation,
    //       crowd systems, particle orientations).

    Quaternion q1 = Quaternion::identity();
    Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), Tolerance::PI / 2.0);
    Quaternion r0 = q1.nlerp(q2, 0.0);
    Quaternion r1 = q1.nlerp(q2, 1.0);

    MINI_CHECK(TOLERANCE.is_close(r0.scalar, q1.scalar));
    MINI_CHECK(TOLERANCE.is_close(r1.scalar, q2.scalar));
}

MINI_TEST("Quaternion", "Json Roundtrip") {
    // uncomment #include "quaternion.h"

    // FUNCTION: q.json_dump / Quaternion::json_load
    // WHAT: Write a quaternion to a JSON file, read it back.
    // WHEN TO USE: Human-readable persistence and debugging, or when sharing
    //       data with tools that consume JSON.

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

    // FUNCTION: q.pb_dump / Quaternion::pb_load
    // WHAT: Write a quaternion to a binary protobuf file, read it back.
    // WHEN TO USE: Compact storage, network protocols, or any pipeline that
    //       needs the smallest serialized footprint and the fastest IO.

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
