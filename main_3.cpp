#include <filesystem>
#include "session.h"
#include "quaternion.h"
#include "plane.h"
#include "vector.h"
#include "point.h"
#include "tolerance.h"
using namespace session_cpp;

static constexpr double PI = Tolerance::PI;

static std::shared_ptr<Plane> make_plane(const Plane& src, const std::string& name, const Color& color) {
    auto p = std::make_shared<Plane>(src);
    p->name = name;
    p->linecolor = color;
    return p;
}

static std::shared_ptr<Plane> identity_plane(const std::string& name) {
    return make_plane(Plane::xy_plane(), name, Color(160, 160, 160));
}

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto out = (base / "session_data" / "QuaternionViz.pb").string();

    Session session("QuaternionViz");
    int gi = 0;

    // 1. Constructor
    {
        auto g = session.add_group("01_Constructor");
        Quaternion q0;
        Quaternion q = Quaternion::from_components(2.0, Vector(1.0, 0.0, 0.0));
        Quaternion qrot = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        Quaternion qmul = qrot * qrot;
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(qrot.get_rotation(), "qrot", Color::blue()), g);
        session.add_plane(make_plane(qmul.get_rotation(), "qrot*qrot", Color::red()), g);
        gi++;
    }

    // 2. Identity
    {
        auto g = session.add_group("02_Identity");
        Quaternion q = Quaternion::identity();
        session.add_plane(make_plane(q.get_rotation(), "identity", Color::blue()), g);
        gi++;
    }

    // 3. From Components — (s, v) are raw quaternion parts, NOT a rotation axis.
    // Demonstrates the gap: blue = quaternion's rotation plane;
    // red  = a plane that ACTUALLY has normal (1,2,3), built directly.
    {
        auto g = session.add_group("03_FromComponents");
        Quaternion q = Quaternion::from_components(2.0, Vector(1.0, 2.0, 3.0));
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.normalized().get_rotation(), "q.normalized rotation", Color::blue()), g);
        Vector n(1.0, 2.0, 3.0);
        Point o(0.0, 0.0, 0.0);
        Plane direct = Plane::from_point_normal(o, n);
        session.add_plane(make_plane(direct, "from_point_normal((1,2,3))", Color::red()), g);
        gi++;
    }

    // 4. From Axis Angle
    {
        auto g = session.add_group("04_FromAxisAngle");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "rot(Z,90)", Color::blue()), g);
        gi++;
    }

    // 5. From Arc
    {
        auto g = session.add_group("05_FromArc");
        Vector src(1.0, 0.0, 0.0);
        Vector dst(0.0, 1.0, 0.0);
        Quaternion q = Quaternion::from_arc(src, dst);
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "from_arc(X,Y)", Color::blue()), g);
        gi++;
    }

    // 6. From Euler
    {
        auto g = session.add_group("06_FromEuler");
        Quaternion q_euler = Quaternion::from_euler(0.0, 0.0, PI / 2.0);
        Quaternion q_axis = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q_euler.get_rotation(), "from_euler", Color::blue()), g);
        session.add_plane(make_plane(q_axis.get_rotation(), "from_axis_angle", Color::red()), g);
        gi++;
    }

    // 7. From Rotation
    {
        auto g = session.add_group("07_FromRotation");
        Plane plane_a = Plane::xy_plane();
        Plane plane_b(Point(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), Vector(-1.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0));
        Quaternion q = Quaternion::from_rotation(plane_a, plane_b);
        session.add_plane(make_plane(plane_a, "plane_a", Color(160, 160, 160)), g);
        session.add_plane(make_plane(plane_b, "plane_b", Color::blue()), g);
        session.add_plane(make_plane(q.get_rotation(), "from_rotation", Color::red()), g);
        gi++;
    }

    // 8. Rotate Vector
    {
        auto g = session.add_group("08_RotateVector");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue()), g);
        gi++;
    }

    // 9. Get Rotation
    {
        auto g = session.add_group("09_GetRotation");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "get_rotation", Color::blue()), g);
        gi++;
    }

    // 10. Magnitude
    {
        auto g = session.add_group("10_Magnitude");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 4.0);
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue()), g);
        gi++;
    }

    // 11. Magnitude Squared
    {
        auto g = session.add_group("11_MagnitudeSquared");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 4.0);
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue()), g);
        gi++;
    }

    // 12. Normalized
    {
        auto g = session.add_group("12_Normalized");
        Quaternion q = Quaternion::from_components(2.0, Vector(0.0, 0.0, 2.0));
        Quaternion n = q.normalized();
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "raw", Color::red()), g);
        session.add_plane(make_plane(n.get_rotation(), "normalized", Color::blue()), g);
        gi++;
    }

    // 13. Conjugate
    {
        auto g = session.add_group("13_Conjugate");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 4.0);
        Quaternion r = q.conjugate();
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue()), g);
        session.add_plane(make_plane(r.get_rotation(), "conjugate", Color::red()), g);
        gi++;
    }

    // 14. Invert
    {
        auto g = session.add_group("14_Invert");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 3.0);
        Quaternion result = q * q.invert();
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue()), g);
        session.add_plane(make_plane(result.get_rotation(), "q*q.invert()", Color::red()), g);
        gi++;
    }

    // 15. Dot
    {
        auto g = session.add_group("15_Dot");
        Quaternion q = Quaternion::identity();
        session.add_plane(make_plane(q.get_rotation(), "identity", Color::blue()), g);
        gi++;
    }

    // 16. Slerp
    {
        auto g = session.add_group("16_Slerp");
        Quaternion q1 = Quaternion::identity();
        Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        Quaternion mid = q1.slerp(q2, 0.5);
        session.add_plane(identity_plane("q1"), g);
        session.add_plane(make_plane(q2.get_rotation(), "q2", Color::red()), g);
        session.add_plane(make_plane(mid.get_rotation(), "slerp(0.5)", Color::blue()), g);
        gi++;
    }

    // 17. Nlerp
    {
        auto g = session.add_group("17_Nlerp");
        Quaternion q1 = Quaternion::identity();
        Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        Quaternion r0 = q1.nlerp(q2, 0.0);
        Quaternion r1 = q1.nlerp(q2, 1.0);
        session.add_plane(make_plane(r0.get_rotation(), "nlerp(0)", Color::blue()), g);
        session.add_plane(make_plane(r1.get_rotation(), "nlerp(1)", Color::red()), g);
        gi++;
    }

    // 18. Json Roundtrip
    {
        auto g = session.add_group("18_JsonRoundtrip");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        q.name = "serialization/test_quaternion";
        std::string fn = (base / "session_data" / "QuaternionViz_q.json").string();
        q.json_dump(fn);
        Quaternion loaded = Quaternion::json_load(fn);
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue()), g);
        session.add_plane(make_plane(loaded.get_rotation(), "json_load", Color::red()), g);
        gi++;
    }

    // 19. Protobuf Roundtrip
    {
        auto g = session.add_group("19_ProtobufRoundtrip");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        q.name = "serialization/test_quaternion";
        std::string fn = (base / "session_data" / "QuaternionViz_q.bin").string();
        q.pb_dump(fn);
        Quaternion loaded = Quaternion::pb_load(fn);
        session.add_plane(identity_plane("identity"), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue()), g);
        session.add_plane(make_plane(loaded.get_rotation(), "pb_load", Color::red()), g);
        gi++;
    }

    session.pb_dump(out);
    fmt::print("Wrote {} groups -> {}\n", gi, out);
    return 0;
}
