#include "mini_test.h"
#include "point.h"
#include "color.h"
#include "xform.h"
#include "tolerance.h"

#include <cmath>
#include <filesystem>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("Point", "Constructor") {
        // uncomment #include "point.h"
        // uncomment #include "vector.h"
        // uncomment #include "color.h"

        // Constructor
        Point p(1.0, 2.0, 3.0);

        // Setters
        p[0] = 10.0;
        p[1] = 20.0;
        p[2] = 30.0;

        // Getters
        double x = p[0];
        double y = p[1];
        double z = p[2];

        // Minimal and Full String Representation
        std::string pstr = p.str();
        std::string prepr = p.repr();

        // Copy (duplicate everything but guid)
        Point pcopy = p;
        Point pother(1.0, 2.0, 3.0);

        // No-copy operators
        Point pmult = p;
        pmult *= 2.0;
        Point pdiv = p;
        pdiv /= 2.0;
        Point padd = p;
        padd += Vector(1.0, 1.0, 1.0);
        Point psub = p;
        psub -= Vector(1.0, 1.0, 1.0);

        // Copy operators
        Point result_mul = p * 2.0;
        Point result_div = p / 2.0;
        Point result_add = p + Vector(1.0, 1.0, 1.0);
        Point diff_point = p - Vector(1.0, 1.0, 1.0);

        // Static sum and sub methods
        Point p1(1.0, 2.0, 3.0);
        Point p2(4.0, 5.0, 6.0);
        Point psum = Point::sum(p1, p2);
        Point pdif = Point::sub(p2, p1);

        MINI_CHECK(
            p.name == "my_point" &&
            p[0] == 10.0 &&
            p[1] == 20.0 &&
            p[2] == 30.0 &&
            p.width == 1.0 &&
            p.pointcolor == Color::blue() &&
            !p.guid.empty()
        );
        MINI_CHECK(x == 10.0 && y == 20.0 && z == 30.0);
        MINI_CHECK(pstr == "10.000000, 20.000000, 30.000000");
        MINI_CHECK(prepr == "Point(my_point, 10.000000, 20.000000, 30.000000, Color(0, 0, 255, 255), 1.000000)");
        MINI_CHECK(pcopy == p && pcopy.guid != p.guid);
        MINI_CHECK(pother != p);
        MINI_CHECK(pmult[0] == 20.0 && pmult[1] == 40.0 && pmult[2] == 60.0);
        MINI_CHECK(pdiv[0] == 5.0 && pdiv[1] == 10.0 && pdiv[2] == 15.0);
        MINI_CHECK(padd[0] == 11.0 && padd[1] == 21.0 && padd[2] == 31.0);
        MINI_CHECK(psub[0] == 9.0 && psub[1] == 19.0 && psub[2] == 29.0);
        MINI_CHECK(result_mul[0] == 20.0 && result_mul[1] == 40.0 && result_mul[2] == 60.0);
        MINI_CHECK(result_div[0] == 5.0 && result_div[1] == 10.0 && result_div[2] == 15.0);
        MINI_CHECK(result_add[0] == 11.0 && result_add[1] == 21.0 && result_add[2] == 31.0);
        MINI_CHECK(diff_point[0] == 9.0 && diff_point[1] == 19.0 && diff_point[2] == 29.0);
        MINI_CHECK(psum[0] == 5.0 && psum[1] == 7.0 && psum[2] == 9.0);
        MINI_CHECK(pdif[0] == 3.0 && pdif[1] == 3.0 && pdif[2] == 3.0);
    }

    MINI_TEST("Point", "Transformation") {
        // uncomment #include "point.h"
        // uncomment #include "xform.h"

        Point p(1.0, 2.0, 3.0);
        p.xform = Xform::translation(1.0, 2.0, 3.0);
        Point p_transformed = p.transformed(); // Make a copy
        p.transform(); // After the call, "xform" is reset

        MINI_CHECK(p_transformed[0] == 2.0 && p_transformed[1] == 4.0 && p_transformed[2] == 6.0);
        MINI_CHECK(p[0] == 2.0 && p[1] == 4.0 && p[2] == 6.0);
        MINI_CHECK(p.xform == Xform::identity());
    }

    MINI_TEST("Point", "Json_roundtrip") {
        // uncomment #include "point.h"
        // uncomment #include "color.h"

        Point p(1.5, 2.5, 3.5);
        p.name = "test_point";
        p.width = 2.0;
        p.pointcolor = Color(255, 128, 64, 255);

        //   jsondump()      │ ordered_json │ to JSON object (internal use)
        //   jsonload(j)     │ ordered_json │ from JSON object (internal use)
        //   json_dumps()    │ std::string  │ to JSON string
        //   json_loads(s)   │ std::string  │ from JSON string
        //   json_dump(path) │ file         │ write to file
        //   json_load(path) │ file         │ read from file

        std::string filename = "serialization/test_point.json";
        p.json_dump(filename);
        Point loaded = Point::json_load(filename);

        MINI_CHECK(loaded.name == p.name);
        MINI_CHECK(loaded[0] == p[0]);
        MINI_CHECK(loaded[1] == p[1]);
        MINI_CHECK(loaded[2] == p[2]);
        MINI_CHECK(loaded.width == p.width);
        MINI_CHECK(loaded.pointcolor.r == 255);
        MINI_CHECK(loaded.pointcolor.g == 128);
        MINI_CHECK(loaded.pointcolor.b == 64);
        MINI_CHECK(loaded.pointcolor.a == 255);

    }

    MINI_TEST("Point", "Protobuf_roundtrip") {
        // uncomment #include "point.h"
        // uncomment #include "color.h"

        Point p(1.5, 2.5, 3.5);
        p.name = "test_point";
        p.width = 2.0;
        p.pointcolor = Color(255, 128, 64, 255);

        std::string filename = "serialization/test_point.bin";
        p.pb_dump(filename);
        Point loaded = Point::pb_load(filename);

        MINI_CHECK(loaded.name == p.name);
        MINI_CHECK(loaded[0] == p[0]);
        MINI_CHECK(loaded[1] == p[1]);
        MINI_CHECK(loaded[2] == p[2]);
        MINI_CHECK(loaded.width == p.width);
        MINI_CHECK(loaded.pointcolor.r == 255);
        MINI_CHECK(loaded.pointcolor.g == 128);
        MINI_CHECK(loaded.pointcolor.b == 64);
        MINI_CHECK(loaded.pointcolor.a == 255);
    }

    MINI_TEST("Point", "Is_ccw") {
        // uncomment #include "point.h"

        Point p0(0.0, 0.0, 0.0);
        Point p1(1.0, 0.0, 0.0);
        Point p2(0.05, 1.0, 0.0);

        // Points must be oriented to xy plane.
        bool is_counter_clock_wise = Point::is_ccw(p0, p1, p2);
        bool is_clock_wise = Point::is_ccw(p2, p1, p0);

        MINI_CHECK(is_counter_clock_wise);
        MINI_CHECK(!is_clock_wise);
    }

    MINI_TEST("Point", "Mid_point") {
        // uncomment #include "point.h"

        Point p0(0.0, 2.0, 1.0);
        Point p1(1.0, 5.0, 3.0);
        Point mid = Point::mid_point(p0, p1);

        MINI_CHECK(mid[0] == 0.5 && mid[1] == 3.5 && mid[2] == 2.0);
    }

    MINI_TEST("Point", "Distance") {
        // uncomment #include "point.h"

        Point p0(0.0, 2.0, 1.0);
        Point p1(1.0, 5.0, 3.0);
        double d = Point::distance(p0, p1);

        MINI_CHECK(TOLERANCE.is_close(d, 3.741657));
    }

    MINI_TEST("Point", "Squared_distance") {
        // uncomment #include "point.h"

        Point p0(0.0, 2.0, 1.0);
        Point p1(1.0, 5.0, 3.0);
        double d = Point::squared_distance(p0, p1);

        MINI_CHECK(TOLERANCE.is_close(d, 14.0));
    }

    MINI_TEST("Point", "Area") {
        // uncomment #include "point.h"

        Point p0(0.0, 0.0, 0.0);
        Point p1(2.0, 0.0, 0.0);
        Point p2(2.0, 2.0, 0.0);
        Point p3(0.0, 2.0, 0.0);
        double area = Point::area({p0, p1, p2, p3});

        MINI_CHECK(area == 4.0);
    }

    MINI_TEST("Point", "Centroid_quad") {
        // uncomment #include "point.h"

        Point p0(0.0, 0.0, 0.0);
        Point p1(2.0, 0.0, 1.0);
        Point p2(2.0, 2.0, 2.0);
        Point p3(0.0, 2.0, 1.0);
        Point centroid = Point::centroid_quad({p0, p1, p2, p3});

        MINI_CHECK(TOLERANCE.is_close(centroid[0], 1.0));
        MINI_CHECK(TOLERANCE.is_close(centroid[1], 1.0));
        MINI_CHECK(TOLERANCE.is_close(centroid[2], 1.0));
    }

}

int main() {
    session_cpp::mini_test::run_all("cpp");
    return 0;
}

