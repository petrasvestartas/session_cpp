#include "mini_test.h"
#include "point.h"
#include "color.h"
#include "xform.h"

using namespace session_cpp::mini_test;


MINI_TEST("Point", "constructor, setters, getters, string, copy, operators") {
    // uncomment #include "point.h"
    // uncomment #include "vector.h"
    // uncomment #include "color.h"
    using namespace session_cpp;

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

    // String representation
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
    Point result_add = p + Vector(1.0, 1.0, 1.0); // Only Vector
    Point diff_point = p - Vector(1.0, 1.0, 1.0); // Only Vector

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
}

int main() {
    session_cpp::mini_test::run_all("cpp");
    return 0;
} 