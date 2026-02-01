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

MINI_TEST("Primitives", "cylinder_surface") {
    NurbsSurface s = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);

    MINI_CHECK(s.is_valid());
    MINI_CHECK(s.is_rational());
    MINI_CHECK(s.cv_count(0) == 9);
    MINI_CHECK(s.cv_count(1) == 2);
    MINI_CHECK(s.order(0) == 3);
    MINI_CHECK(s.order(1) == 2);

    Point p00 = s.point_at(0.0, 0.0);
    MINI_CHECK(std::abs(p00[0] - 1.0) < 1e-10);
    MINI_CHECK(std::abs(p00[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p00[2] - 0.0) < 1e-10);

    Point p01 = s.point_at(0.0, 1.0);
    MINI_CHECK(std::abs(p01[0] - 1.0) < 1e-10);
    MINI_CHECK(std::abs(p01[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p01[2] - 5.0) < 1e-10);

    Point pmid = s.point_at(1.0, 0.5);
    MINI_CHECK(std::abs(pmid[0] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(pmid[1] - 1.0) < 1e-10);
    MINI_CHECK(std::abs(pmid[2] - 2.5) < 1e-10);
}

MINI_TEST("Primitives", "cone_surface") {
    NurbsSurface s = Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0);

    MINI_CHECK(s.is_valid());
    MINI_CHECK(s.is_rational());
    MINI_CHECK(s.cv_count(0) == 9);
    MINI_CHECK(s.cv_count(1) == 2);
    MINI_CHECK(s.order(0) == 3);
    MINI_CHECK(s.order(1) == 2);

    Point pbase = s.point_at(0.0, 0.0);
    MINI_CHECK(std::abs(pbase[0] - 1.0) < 1e-10);
    MINI_CHECK(std::abs(pbase[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(pbase[2] - 0.0) < 1e-10);

    Point papex = s.point_at(0.0, 1.0);
    MINI_CHECK(std::abs(papex[0] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(papex[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(papex[2] - 5.0) < 1e-10);

    Point pmid = s.point_at(0.0, 0.5);
    MINI_CHECK(std::abs(pmid[0] - 0.5) < 1e-10);
    MINI_CHECK(std::abs(pmid[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(pmid[2] - 2.5) < 1e-10);
}

MINI_TEST("Primitives", "torus_surface") {
    NurbsSurface s = Primitives::torus_surface(0.0, 0.0, 0.0, 3.0, 1.0);

    MINI_CHECK(s.is_valid());
    MINI_CHECK(s.is_rational());
    MINI_CHECK(s.cv_count(0) == 9);
    MINI_CHECK(s.cv_count(1) == 9);
    MINI_CHECK(s.order(0) == 3);
    MINI_CHECK(s.order(1) == 3);

    Point p00 = s.point_at(0.0, 0.0);
    MINI_CHECK(std::abs(p00[0] - 4.0) < 1e-10);
    MINI_CHECK(std::abs(p00[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p00[2] - 0.0) < 1e-10);

    Point p10 = s.point_at(1.0, 0.0);
    MINI_CHECK(std::abs(p10[0] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p10[1] - 4.0) < 1e-10);
    MINI_CHECK(std::abs(p10[2] - 0.0) < 1e-10);

    Point p_top = s.point_at(0.0, 1.0);
    MINI_CHECK(std::abs(p_top[0] - 3.0) < 1e-10);
    MINI_CHECK(std::abs(p_top[1] - 0.0) < 1e-10);
    MINI_CHECK(std::abs(p_top[2] - 1.0) < 1e-10);
}

} // namespace session_cpp
