#include "mini_test.h"
#include "point.h"
#include "color.h"
#include "vector.h"
#include "xform.h"
#include "tolerance.h"

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Point", "Constructor") {
        Point p(1.0, 2.0, 3.0);

        p[0] = 10.0;
        p[1] = 20.0;
        p[2] = 30.0;

        double x = p[0];
        double y = p[1];
        double z = p[2];

        std::string pstr = p.str();
        std::string prepr = p.repr();

        Point pcopy = p;
        Point pother(1.0, 2.0, 3.0);

        Point pmult = p;
        pmult *= 2.0;
        Point pdiv = p;
        pdiv /= 2.0;
        Point padd = p;
        padd += Vector(1.0, 1.0, 1.0);
        Point psub = p;
        psub -= Vector(1.0, 1.0, 1.0);

        Point result_mul = p * 2.0;
        Point result_div = p / 2.0;
        Point result_add = p + Vector(1.0, 1.0, 1.0);
        Point result_sub = p - Vector(1.0, 1.0, 1.0);
        Vector result_diff = p - pother;

        Point p1(1.0, 2.0, 3.0);
        Point p2(4.0, 5.0, 6.0);
        Point psum = Point::sum(p1, p2);
        Point pdif = Point::sub(p2, p1);

        MINI_CHECK(p.name == "my_point");
        MINI_CHECK(p[0] == 10.0 && p[1] == 20.0 && p[2] == 30.0);
        MINI_CHECK(p.width == 1.0);
        MINI_CHECK(p.pointcolor == Color::black());
        MINI_CHECK(p.guid() != "");
        MINI_CHECK(x == 10.0 && y == 20.0 && z == 30.0);
        MINI_CHECK(pstr == "10.000000, 20.000000, 30.000000");
        MINI_CHECK(prepr == "Point(my_point, 10.000000, 20.000000, 30.000000, Color(black, 0.0, 0.0, 0.0, 1.0), 1.000000)");
        MINI_CHECK(pcopy == p && pcopy.guid() != p.guid());
        MINI_CHECK(pother != p);
        MINI_CHECK(pmult[0] == 20.0 && pmult[1] == 40.0 && pmult[2] == 60.0);
        MINI_CHECK(pdiv[0] == 5.0 && pdiv[1] == 10.0 && pdiv[2] == 15.0);
        MINI_CHECK(padd[0] == 11.0 && padd[1] == 21.0 && padd[2] == 31.0);
        MINI_CHECK(psub[0] == 9.0 && psub[1] == 19.0 && psub[2] == 29.0);
        MINI_CHECK(result_mul[0] == 20.0 && result_mul[1] == 40.0 && result_mul[2] == 60.0);
        MINI_CHECK(result_div[0] == 5.0 && result_div[1] == 10.0 && result_div[2] == 15.0);
        MINI_CHECK(result_add[0] == 11.0 && result_add[1] == 21.0 && result_add[2] == 31.0);
        MINI_CHECK(result_sub[0] == 9.0 && result_sub[1] == 19.0 && result_sub[2] == 29.0);
        MINI_CHECK(result_diff[0] == 9.0 && result_diff[1] == 18.0 && result_diff[2] == 27.0);
        MINI_CHECK(psum[0] == 5.0 && psum[1] == 7.0 && psum[2] == 9.0);
        MINI_CHECK(pdif[0] == 3.0 && pdif[1] == 3.0 && pdif[2] == 3.0);
    }

    MINI_TEST("Point", "Transformation") {
        Point p(1.0, 2.0, 3.0);
        Xform xform = Xform::translation(1.0, 2.0, 3.0);
        Point moved = p.transformed(xform);
        p.transform(xform);

        MINI_CHECK(moved[0] == 2.0 && moved[1] == 4.0 && moved[2] == 6.0);
        MINI_CHECK(p[0] == 2.0 && p[1] == 4.0 && p[2] == 6.0);
    }

    MINI_TEST("Point", "Json Roundtrip") {
        Point p(1.5, 2.5, 3.5, "test_point");
        p.width = 2.0;
        p.pointcolor = Color(1.0f, 0.5f, 0.25f, 1.0f);

        std::string filename = "serialization/test_point.json";
        p.file_json_dump(filename);
        Point loaded = Point::file_json_load(filename);

        MINI_CHECK(loaded.name == "test_point");
        MINI_CHECK(loaded[0] == 1.5 && loaded[1] == 2.5 && loaded[2] == 3.5);
        MINI_CHECK(loaded.width == 2.0);
        MINI_CHECK(loaded.pointcolor[0] == 1.0f);
        MINI_CHECK(loaded.pointcolor[1] == 0.5f);
        MINI_CHECK(loaded.pointcolor[2] == 0.25f);
        MINI_CHECK(loaded.pointcolor[3] == 1.0f);
    }

    MINI_TEST("Point", "Protobuf Roundtrip") {
        Point p(1.5, 2.5, 3.5, "test_point");
        p.width = 2.0;
        p.pointcolor = Color(1.0f, 0.5f, 0.25f, 1.0f);

        std::string filename = "serialization/test_point.bin";
        p.pb_dump(filename);
        Point loaded = Point::pb_load(filename);

        MINI_CHECK(loaded.name == "test_point");
        MINI_CHECK(loaded[0] == 1.5 && loaded[1] == 2.5 && loaded[2] == 3.5);
        MINI_CHECK(loaded.width == 2.0);
        MINI_CHECK(loaded.pointcolor[0] == 1.0f);
        MINI_CHECK(loaded.pointcolor[1] == 0.5f);
        MINI_CHECK(loaded.pointcolor[2] == 0.25f);
        MINI_CHECK(loaded.pointcolor[3] == 1.0f);
    }

    MINI_TEST("Point", "Is Ccw") {
        Point p0(0.0, 0.0, 0.0);
        Point p1(1.0, 0.0, 0.0);
        Point p2(0.05, 1.0, 0.0);
        bool ccw = Point::is_ccw(p0, p1, p2);
        bool cw = Point::is_ccw(p2, p1, p0);

        MINI_CHECK(ccw);
        MINI_CHECK(!cw);
    }

    MINI_TEST("Point", "Mid Point") {
        Point p0(0.0, 2.0, 1.0);
        Point p1(1.0, 5.0, 3.0);
        Point mid = Point::mid_point(p0, p1);

        MINI_CHECK(mid[0] == 0.5 && mid[1] == 3.5 && mid[2] == 2.0);
    }

    MINI_TEST("Point", "Distance") {
        Point p0(0.0, 2.0, 1.0);
        Point p1(1.0, 5.0, 3.0);
        double d = Point::distance(p0, p1);

        MINI_CHECK(TOLERANCE.is_close(d, 3.741657));
    }

    MINI_TEST("Point", "Squared Distance") {
        Point p0(0.0, 2.0, 1.0);
        Point p1(1.0, 5.0, 3.0);
        double d = Point::squared_distance(p0, p1);

        MINI_CHECK(TOLERANCE.is_close(d, 14.0));
    }

    MINI_TEST("Point", "Interpolate") {
        Point a(0.0, 0.0, 0.0);
        Point b(4.0, 8.0, 12.0);
        Point half = Point::lerp(a, b, 0.5);
        std::vector<Point> inner = Point::interpolate(a, b, 3);
        std::vector<Point> both = Point::interpolate(a, b, 3, 1);
        std::vector<Point> start = Point::interpolate(a, b, 3, 2);

        MINI_CHECK(half[0] == 2.0 && half[1] == 4.0 && half[2] == 6.0);
        MINI_CHECK(inner.size() == 3);
        MINI_CHECK(inner[0][0] == 1.0 && inner[1][0] == 2.0 && inner[2][0] == 3.0);
        MINI_CHECK(both.size() == 5);
        MINI_CHECK(both[0][0] == 0.0 && both[4][0] == 4.0);
        MINI_CHECK(start.size() == 4);
        MINI_CHECK(start[0][0] == 0.0 && start[3][0] == 3.0);
    }

    MINI_TEST("Point", "Area") {
        Point p0(0.0, 0.0, 0.0);
        Point p1(2.0, 0.0, 0.0);
        Point p2(2.0, 2.0, 0.0);
        Point p3(0.0, 2.0, 0.0);
        double area = Point::area({p0, p1, p2, p3});

        MINI_CHECK(area == 4.0);
    }

    MINI_TEST("Point", "Centroid Quad") {
        Point p0(0.0, 0.0, 0.0);
        Point p1(2.0, 0.0, 1.0);
        Point p2(2.0, 2.0, 2.0);
        Point p3(0.0, 2.0, 1.0);
        Point centroid = Point::centroid_quad({p0, p1, p2, p3});

        MINI_CHECK(TOLERANCE.is_close(centroid[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(centroid[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(centroid[2], 1.0));
    }

    MINI_TEST("Point", "Centroid") {
        Point p0(0.0, 0.0, 0.0);
        Point p1(2.0, 0.0, 0.0);
        Point p2(2.0, 2.0, 0.0);
        Point p3(0.0, 2.0, 0.0);
        Point centroid = Point::centroid({p0, p1, p2, p3});

        MINI_CHECK(TOLERANCE.is_close(centroid[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(centroid[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(centroid[2], 0.0));
    }

    MINI_TEST("Point", "Dihedral Angle Deg") {
        Point p(0.0, 0.0, 0.0);
        Point q(1.0, 0.0, 0.0);
        Point r(0.0, 1.0, 0.0);
        Point s(0.0, 0.0, 1.0);
        double angle = Point::dihedral_angle_deg(p, q, r, s);

        MINI_CHECK(TOLERANCE.is_close(angle, 90.0));
    }

}

int main() {
    return session_cpp::mini_test::run_all("cpp") ? 1 : 0;
}
