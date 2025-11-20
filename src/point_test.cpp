#include "mini_test.h"
#include "point.h"

using namespace session_cpp;
using namespace session_cpp::mini_test;

// Mini tests mirroring the Python mini_test.py structure

MINI_TEST(Point, constructor) {

    #include "point.h"
    Point p(1.0, 2.0, 3.0);
    p[0] = 10.0;

    MINI_CHECK(p.name == "my_point");
    MINI_CHECK(!p.guid.empty());
    MINI_CHECK(p[0] == 10.0);
    MINI_CHECK(p[1] == 2.0);
    MINI_CHECK(p[2] == 3.0);
    MINI_CHECK(p.width == 1.0);
    MINI_CHECK(p.pointcolor == Color::white());
}

MINI_TEST(Point, equality_equal) {
    Point p1(1.0, 2.0, 3.0);
    Point p2(1.0, 2.0, 3.0);

    bool eq_result = (p1 == p2);
    bool neq_result = (p1 != p2);

    MINI_CHECK(eq_result == true);
    MINI_CHECK(neq_result == false);
}

MINI_TEST(Point, equality_not_equal) {
    Point p3(1.0, 2.0, 3.0);
    Point p4(1.1, 2.0, 3.0);

    bool eq_result = (p3 == p4);
    bool neq_result = (p3 != p4);

    MINI_CHECK(eq_result == false);
    MINI_CHECK(neq_result == true);
}

int main() {
    mini_test::run_all("cpp");
    return 0;
}
