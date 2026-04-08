#include <filesystem>
#include "session.h"
#include "quaternion.h"
#include "plane.h"
#include "vector.h"
#include "point.h"
#include "line.h"
#include "polyline.h"
#include "tolerance.h"
using namespace session_cpp;

// Visualize quaternions as planes — one group per MINI_TEST in quaternion_test.cpp.
// Each group is shifted on +X so they sit side-by-side in the viewer.
// Run, then load session_data/QuaternionViz.pb in the Vue viewer.

static constexpr double STEP = 5.0;
static constexpr double PI = Tolerance::PI;

static std::shared_ptr<Plane> make_plane(const Plane& src, const std::string& name, const Color& color, double dx) {
    auto p = std::make_shared<Plane>(src);
    *p += Vector(dx, 0.0, 0.0);
    p->name = name;
    p->linecolor = color;
    return p;
}

static std::shared_ptr<Plane> identity_plane(const std::string& name, double dx) {
    return make_plane(Plane::xy_plane(), name, Color(160, 160, 160), dx);
}

static std::shared_ptr<Line> shifted_line(const Point& a, const Point& b, const std::string& name, const Color& color, double dx) {
    auto l = std::make_shared<Line>(a[0] + dx, a[1], a[2], b[0] + dx, b[1], b[2]);
    l->name = name;
    l->linecolor = color;
    return l;
}

static std::shared_ptr<Polyline> shifted_polyline(const std::vector<Point>& pts, const std::string& name, const Color& color, double dx) {
    std::vector<Point> moved;
    moved.reserve(pts.size());
    for (auto& p : pts) moved.emplace_back(p[0] + dx, p[1], p[2]);
    auto pl = std::make_shared<Polyline>(moved);
    pl->name = name;
    pl->linecolor = color;
    return pl;
}

int main() {
    auto base = std::filesystem::path(__FILE__).parent_path().parent_path();
    auto out = (base / "session_data" / "QuaternionViz.pb").string();

    Session session("QuaternionViz");
    int gi = 0;

    // 1. Constructor — identity, q90 (Z by π/2), q180 (q90*q90)
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("01_Constructor");
        Quaternion q90 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        Quaternion q180 = q90 * q90;
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(q90.get_rotation(), "q90", Color::blue(), dx), g);
        session.add_plane(make_plane(q180.get_rotation(), "q90*q90 = q180", Color::red(), dx), g);
    }

    // 2. Identity — single frame, the do-nothing rotation
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("02_Identity");
        session.add_plane(make_plane(Quaternion::identity().get_rotation(), "identity", Color::blue(), dx), g);
    }

    // 3. From Scalar And Vector — raw quaternion is not unit; show normalized form
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("03_FromScalarVector");
        Quaternion raw = Quaternion::from_scalar_and_vector(2.0, Vector(1.0, 2.0, 3.0));
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(raw.normalized().get_rotation(), "raw.normalized()", Color::blue(), dx), g);
    }

    // 4. From Axis Angle — show the rotation axis as a literal line
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("04_FromAxisAngle");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(q.get_rotation(), "rot(Z, 90deg)", Color::blue(), dx), g);
        session.add_line(shifted_line(Point(0.0, 0.0, -1.5), Point(0.0, 0.0, 1.5), "axis_Z", Color::red(), dx), g);
    }

    // 5. From Arc — sample the great-circle path src->dst on the unit circle
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("05_FromArc");
        Vector src(1.0, 0.0, 0.0);
        Vector dst(0.0, 1.0, 0.0);
        Quaternion q = Quaternion::from_arc(src, dst);
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(q.get_rotation(), "from_arc(X,Y)", Color::blue(), dx), g);
        std::vector<Point> arc;
        for (int i = 0; i <= 20; i++) {
            double t = double(i) / 20.0 * (PI / 2.0);
            arc.emplace_back(std::cos(t), std::sin(t), 0.0);
        }
        session.add_polyline(shifted_polyline(arc, "arc_X_to_Y", Color::red(), dx), g);
    }

    // 6. From Euler — equivalent to axis-angle around Z
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("06_FromEuler");
        Quaternion q_euler = Quaternion::from_euler(0.0, 0.0, PI / 2.0);
        Quaternion q_axis = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(q_euler.get_rotation(), "from_euler(0,0,90)", Color::blue(), dx), g);
        session.add_plane(make_plane(q_axis.get_rotation(), "from_axis_angle(Z,90)", Color::red(), dx), g);
    }

    // 7. From Rotation — quaternion that maps plane_a -> plane_b
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("07_FromRotation");
        Plane plane_a = Plane::xy_plane();
        Plane plane_b(Point(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), Vector(-1.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0));
        Quaternion q = Quaternion::from_rotation(plane_a, plane_b);
        session.add_plane(make_plane(plane_a, "plane_a", Color(160, 160, 160), dx), g);
        session.add_plane(make_plane(plane_b, "plane_b", Color::blue(), dx), g);
        session.add_plane(make_plane(q.get_rotation(), "from_rotation(a,b)", Color::red(), dx), g);
    }

    // 8. Rotate Vector — input X, output Y
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("08_RotateVector");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        Vector in(1.0, 0.0, 0.0);
        Vector out = q.rotate_vector(in);
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue(), dx), g);
        session.add_line(shifted_line(Point(0.0, 0.0, 0.0), Point(in[0], in[1], in[2]), "input", Color(160, 160, 160), dx), g);
        session.add_line(shifted_line(Point(0.0, 0.0, 0.0), Point(out[0], out[1], out[2]), "rotated", Color::red(), dx), g);
    }

    // 9. Get Rotation — the canonical example: q -> Plane
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("09_GetRotation");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(q.get_rotation(), "q.get_rotation()", Color::blue(), dx), g);
    }

    // 10. Magnitude — unit quaternion is a valid rotation, non-unit is not
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("10_Magnitude");
        Quaternion qu = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 4.0);
        Quaternion qbad = qu * 2.0;
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(qu.get_rotation(), "|q|=1 valid", Color::blue(), dx), g);
        session.add_plane(make_plane(qbad.get_rotation(), "|q|=2 invalid", Color::red(), dx), g);
    }

    // 11. Magnitude Squared — same scene as #10 (mag^2 is just an optimization)
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("11_MagnitudeSquared");
        Quaternion qu = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 4.0);
        Quaternion qbad = qu * 2.0;
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(qu.get_rotation(), "|q|^2=1 valid", Color::blue(), dx), g);
        session.add_plane(make_plane(qbad.get_rotation(), "|q|^2=4 invalid", Color::red(), dx), g);
    }

    // 12. Normalized — before vs after normalize, side-by-side
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("12_Normalized");
        Quaternion raw = Quaternion::from_scalar_and_vector(2.0, Vector(0.0, 0.0, 2.0));
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(raw.get_rotation(), "raw skewed", Color::red(), dx), g);
        session.add_plane(make_plane(raw.normalized().get_rotation(), "normalized", Color::blue(), dx), g);
    }

    // 13. Conjugate — the inverse rotation (mirror about the rotation axis)
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("13_Conjugate");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 4.0);
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue(), dx), g);
        session.add_plane(make_plane(q.conjugate().get_rotation(), "q.conjugate()", Color::red(), dx), g);
    }

    // 14. Invert — q * q.invert() must overlay identity exactly
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("14_Invert");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 3.0);
        Quaternion qid = q * q.invert();
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue(), dx), g);
        session.add_plane(make_plane(qid.get_rotation(), "q*q.invert()", Color::red(), dx), g);
    }

    // 15. Dot — algebraic, not geometric. Print result, show single frame.
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("15_Dot");
        Quaternion qid = Quaternion::identity();
        fmt::print("dot(identity, identity) = {}\n", qid.dot(qid));
        session.add_plane(make_plane(qid.get_rotation(), "identity", Color::blue(), dx), g);
    }

    // 16. Slerp — 11 frames sampled along q1 -> q2 (smooth angular sweep)
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("16_Slerp");
        Quaternion q1 = Quaternion::identity();
        Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        for (int i = 0; i <= 10; i++) {
            double t = double(i) / 10.0;
            Quaternion qt = q1.slerp(q2, t);
            session.add_plane(make_plane(qt.get_rotation(), "slerp_" + std::to_string(i), Color::blue(), dx), g);
        }
    }

    // 17. Nlerp — same endpoints, slightly uneven middle (place beside #16 to compare)
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("17_Nlerp");
        Quaternion q1 = Quaternion::identity();
        Quaternion q2 = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        for (int i = 0; i <= 10; i++) {
            double t = double(i) / 10.0;
            Quaternion qt = q1.nlerp(q2, t);
            session.add_plane(make_plane(qt.get_rotation(), "nlerp_" + std::to_string(i), Color::red(), dx), g);
        }
    }

    // 18. Json Roundtrip — q and json-loaded q should overlay perfectly
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("18_JsonRoundtrip");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        q.name = "serialization/test_quaternion";
        std::string fn = (base / "session_data" / "QuaternionViz_q.json").string();
        q.json_dump(fn);
        Quaternion loaded = Quaternion::json_load(fn);
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue(), dx), g);
        session.add_plane(make_plane(loaded.get_rotation(), "json_load(q)", Color::red(), dx), g);
    }

    // 19. Protobuf Roundtrip — same as #18 but with pb_dump/pb_load
    {
        double dx = gi++ * STEP;
        auto g = session.add_group("19_ProtobufRoundtrip");
        Quaternion q = Quaternion::from_axis_angle(Vector(0.0, 0.0, 1.0), PI / 2.0);
        q.name = "serialization/test_quaternion";
        std::string fn = (base / "session_data" / "QuaternionViz_q.bin").string();
        q.pb_dump(fn);
        Quaternion loaded = Quaternion::pb_load(fn);
        session.add_plane(identity_plane("identity", dx), g);
        session.add_plane(make_plane(q.get_rotation(), "q", Color::blue(), dx), g);
        session.add_plane(make_plane(loaded.get_rotation(), "pb_load(q)", Color::red(), dx), g);
    }

    session.pb_dump(out);
    fmt::print("Wrote {} groups -> {}\n", gi, out);
    return 0;
}
