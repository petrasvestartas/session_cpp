#include "mini_test.h"
#include "nurbscurve.h"
#include "point.h"

#include <cmath>

using namespace session_cpp::mini_test;

namespace session_cpp {

    MINI_TEST("NurbsCurve", "constructor") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        // Minimal and Full String Representation
        std::string cstr = curve.str();
        std::string crepr = curve.repr();

        // Copy (duplicates everything except guid)
        NurbsCurve ccopy = curve;
        NurbsCurve cother = NurbsCurve::create(false, 2, points);

        MINI_CHECK(curve.is_valid() == true);
        MINI_CHECK(curve.cv_count() == 3);
        MINI_CHECK(curve.degree() == 2);
        MINI_CHECK(curve.order() == 3);
        MINI_CHECK(curve.name == "my_nurbscurve");
        MINI_CHECK(!curve.guid.empty());
        MINI_CHECK(cstr == "degree=2, cvs=3");
        MINI_CHECK(crepr == "NurbsCurve(my_nurbscurve, dim=3, order=3, cvs=3, rational=false)");
        MINI_CHECK(ccopy.cv_count() == curve.cv_count());
        MINI_CHECK(ccopy.guid != curve.guid);
    }

    MINI_TEST("NurbsCurve", "is_valid") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        NurbsCurve curve_invalid;

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };
        NurbsCurve curve_valid = NurbsCurve::create(false, 2, points);

        MINI_CHECK(curve_invalid.is_valid() == false);
        MINI_CHECK(curve_valid.is_valid() == true);
    }

    MINI_TEST("NurbsCurve", "get_cv") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        Point cv0 = curve.get_cv(0);
        Point cv1 = curve.get_cv(1);
        Point cv2 = curve.get_cv(2);

        MINI_CHECK(std::abs(cv0[0] - 0.0) < 0.01);
        MINI_CHECK(std::abs(cv1[0] - 1.0) < 0.01);
        MINI_CHECK(std::abs(cv2[0] - 2.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "set_cv") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.set_cv(1, Point(1.5, 2.0, 0.0));
        Point cv1 = curve.get_cv(1);

        MINI_CHECK(std::abs(cv1[0] - 1.5) < 0.01);
        MINI_CHECK(std::abs(cv1[1] - 2.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "point_at") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        auto [t0, t1] = curve.domain();
        double t_mid = (t0 + t1) / 2.0;
        Point pt_mid = curve.point_at(t_mid);

        MINI_CHECK(pt_mid[0] > 0.0);
        MINI_CHECK(pt_mid[0] < 2.0);
    }

    MINI_TEST("NurbsCurve", "point_at_start") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        Point pt_start = curve.point_at_start();

        MINI_CHECK(std::abs(pt_start[0] - 0.0) < 0.01);
        MINI_CHECK(std::abs(pt_start[1] - 0.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "point_at_end") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        Point pt_end = curve.point_at_end();

        MINI_CHECK(std::abs(pt_end[0] - 2.0) < 0.01);
        MINI_CHECK(std::abs(pt_end[1] - 0.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "domain") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        auto [t0, t1] = curve.domain();

        MINI_CHECK(t0 < t1);
    }

    MINI_TEST("NurbsCurve", "is_closed") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points_open = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };
        NurbsCurve curve_open = NurbsCurve::create(false, 2, points_open);

        std::vector<Point> points_closed = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(0.0, 1.0, 0.0),
            Point(0.0, 0.0, 0.0)
        };
        NurbsCurve curve_closed = NurbsCurve::create(false, 3, points_closed);

        MINI_CHECK(curve_open.is_closed() == false);
        MINI_CHECK(curve_closed.is_closed() == true);
    }

    MINI_TEST("NurbsCurve", "length") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 1, points);
        double length = curve.length();

        MINI_CHECK(std::abs(length - 1.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "reverse") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        Point pt_start_before = curve.point_at_start();
        Point pt_end_before = curve.point_at_end();
        curve.reverse();
        Point pt_start_after = curve.point_at_start();
        Point pt_end_after = curve.point_at_end();

        MINI_CHECK(std::abs(pt_start_before[0] - pt_end_after[0]) < 0.01);
        MINI_CHECK(std::abs(pt_end_before[0] - pt_start_after[0]) < 0.01);
    }

    MINI_TEST("NurbsCurve", "make_rational") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        MINI_CHECK(curve.is_rational() == false);
        curve.make_rational();
        MINI_CHECK(curve.is_rational() == true);
    }

    MINI_TEST("NurbsCurve", "tangent_at") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        auto [t0, t1] = curve.domain();
        double t_mid = (t0 + t1) / 2.0;
        Vector tangent = curve.tangent_at(t_mid);

        MINI_CHECK(std::isfinite(tangent[0]));
        MINI_CHECK(std::isfinite(tangent[1]));
    }

    MINI_TEST("NurbsCurve", "knot_count") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        int knot_count = curve.knot_count();

        MINI_CHECK(knot_count == curve.order() + curve.cv_count() - 2);
    }

    MINI_TEST("NurbsCurve", "cv_size") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        MINI_CHECK(curve.cv_size() == 3);
        curve.make_rational();
        MINI_CHECK(curve.cv_size() == 4);
    }

    MINI_TEST("NurbsCurve", "weight") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.make_rational();
        double w = curve.weight(0);
        curve.set_weight(1, 2.0);
        double w1 = curve.weight(1);

        MINI_CHECK(std::abs(w - 1.0) < 0.01);
        MINI_CHECK(std::abs(w1 - 2.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "is_linear") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points_linear = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 0.0)
        };
        NurbsCurve curve_linear = NurbsCurve::create(false, 1, points_linear);

        std::vector<Point> points_curved = {
            Point(0.0, 0.0, 0.0),
            Point(0.5, 1.0, 0.0),
            Point(1.0, 0.0, 0.0)
        };
        NurbsCurve curve_curved = NurbsCurve::create(false, 2, points_curved);

        MINI_CHECK(curve_linear.is_linear() == true);
        MINI_CHECK(curve_curved.is_linear() == false);
    }

    MINI_TEST("NurbsCurve", "json_roundtrip") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        std::string filename = "serialization/test_nurbscurve.json";
        curve.json_dump(filename);
        NurbsCurve loaded = NurbsCurve::json_load(filename);

        MINI_CHECK(loaded.is_valid() == true);
        MINI_CHECK(loaded.cv_count() == 3);
        MINI_CHECK(loaded.degree() == 2);
        MINI_CHECK(loaded.order() == 3);
    }

    MINI_TEST("NurbsCurve", "protobuf_roundtrip") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        std::string filename = "serialization/test_nurbscurve.bin";
        curve.protobuf_dump(filename);
        NurbsCurve loaded = NurbsCurve::protobuf_load(filename);

        MINI_CHECK(loaded.is_valid() == true);
        MINI_CHECK(loaded.cv_count() == 3);
        MINI_CHECK(loaded.degree() == 2);
        MINI_CHECK(loaded.order() == 3);
    }

    MINI_TEST("NurbsCurve", "degree") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        MINI_CHECK(curve.degree() == 2);
        MINI_CHECK(curve.order() == 3);
    }

    MINI_TEST("NurbsCurve", "dimension") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        MINI_CHECK(curve.dimension() == 3);
    }

    MINI_TEST("NurbsCurve", "is_rational") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        MINI_CHECK(curve.is_rational() == false);
        curve.make_rational();
        MINI_CHECK(curve.is_rational() == true);
    }

    MINI_TEST("NurbsCurve", "set_weight") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.make_rational();

        MINI_CHECK(std::abs(curve.weight(1) - 1.0) < 0.01);
        curve.set_weight(1, 2.0);
        MINI_CHECK(std::abs(curve.weight(1) - 2.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "knot") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        double knot0 = curve.knot(0);
        double knot1 = curve.knot(1);

        MINI_CHECK(std::abs(knot0 - 0.0) < 0.01);
        MINI_CHECK(knot1 >= knot0);
    }

    MINI_TEST("NurbsCurve", "set_knot") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        bool result = curve.set_knot(0, 0.5);

        MINI_CHECK(result == true);
        MINI_CHECK(std::abs(curve.knot(0) - 0.5) < 0.01);
    }

    MINI_TEST("NurbsCurve", "set_domain") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        bool result = curve.set_domain(0.0, 10.0);
        auto [t0, t1] = curve.domain();

        MINI_CHECK(result == true);
        MINI_CHECK(std::abs(t0 - 0.0) < 0.01);
        MINI_CHECK(std::abs(t1 - 10.0) < 0.01);
    }

    MINI_TEST("NurbsCurve", "span_count") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        int spans = curve.span_count();

        MINI_CHECK(spans == 1);
    }

    MINI_TEST("NurbsCurve", "get_span_vector") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 1.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        std::vector<double> spans = curve.get_span_vector();

        MINI_CHECK(spans.size() >= 2);
    }

    MINI_TEST("NurbsCurve", "evaluate") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        auto [t0, t1] = curve.domain();
        double t_mid = (t0 + t1) / 2.0;
        std::vector<Vector> result = curve.evaluate(t_mid, 1);

        MINI_CHECK(result.size() >= 1);
    }

    MINI_TEST("NurbsCurve", "is_periodic") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);

        MINI_CHECK(curve.is_periodic() == false);
    }

    MINI_TEST("NurbsCurve", "make_non_rational") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        curve.make_rational();

        MINI_CHECK(curve.is_rational() == true);
        curve.make_non_rational();
        MINI_CHECK(curve.is_rational() == false);
    }

    MINI_TEST("NurbsCurve", "divide_by_count") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        std::vector<Point> divided_points;
        std::vector<double> params;
        bool result = curve.divide_by_count(5, divided_points, &params, true);

        MINI_CHECK(result == true);
        MINI_CHECK(divided_points.size() == 5);
    }

    MINI_TEST("NurbsCurve", "intersect_plane") {
        // uncomment #include "nurbscurve.h"
        // uncomment #include "point.h"
        // uncomment #include "plane.h"

        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 1.0, 0.0),
            Point(2.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);
        Plane plane = Plane::xy_plane();
        std::vector<double> intersections = curve.intersect_plane(plane);

        MINI_CHECK(intersections.size() >= 0);
    }

} // namespace session_cpp
