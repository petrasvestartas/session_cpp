#include "mini_test.h"
#include "primitives.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

MINI_TEST("Primitives", "circle") {
    NurbsCurve c = Primitives::circle(0.0, 0.0, 0.0, 1.0);

    MINI_CHECK(c.cv_count() == 9);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == true);
}

MINI_TEST("Primitives", "ellipse") {
    NurbsCurve c = Primitives::ellipse(0.0, 0.0, 0.0, 2.0, 1.0);

    MINI_CHECK(c.cv_count() == 9);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == true);
}

MINI_TEST("Primitives", "arc") {
    Point start(0.0, 0.0, 0.0);
    Point mid(1.0, 1.0, 0.0);
    Point end(2.0, 0.0, 0.0);
    NurbsCurve c = Primitives::arc(start, mid, end);

    MINI_CHECK(c.cv_count() == 3);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == true);
}

MINI_TEST("Primitives", "parabola") {
    Point p0(-1.0, 1.0, 0.0);
    Point p1(0.0, 0.0, 0.0);
    Point p2(1.0, 1.0, 0.0);
    NurbsCurve c = Primitives::parabola(p0, p1, p2);

    MINI_CHECK(c.cv_count() == 3);
    MINI_CHECK(c.order() == 3);
    MINI_CHECK(c.is_rational() == false);
}

MINI_TEST("Primitives", "hyperbola") {
    Point center(0.0, 0.0, 0.0);
    NurbsCurve c = Primitives::hyperbola(center, 1.0, 1.0, 1.0);

    MINI_CHECK(c.cv_count() >= 4);
    MINI_CHECK(c.order() == 4);
    MINI_CHECK(c.is_rational() == false);
}

MINI_TEST("Primitives", "spiral") {
    NurbsCurve c = Primitives::spiral(1.0, 2.0, 1.0, 5.0);

    MINI_CHECK(c.cv_count() >= 4);
    MINI_CHECK(c.order() == 4);
    MINI_CHECK(c.is_rational() == false);
}

} // namespace session_cpp
