#include "src/nurbscurve.h"
#include "src/point.h"
#include "src/vector.h"
#include <iostream>
#include <iomanip>

using namespace session_cpp;

int main() {
        std::vector<Point> points = {
            Point(0.0, 0.0, 0.0),
            Point(1.0, 2.0, 0.0),
            Point(2.0, 0.0, 0.0),
            Point(3.0, 2.0, 0.0),
            Point(4.0, 0.0, 0.0)
        };

        NurbsCurve curve = NurbsCurve::create(false, 2, points);


        // Reverse the curve
        NurbsCurve curve_reversed = curve;
        curve_reversed.reverse();
        // MINI_CHECK(TOLERANCE.is_point_close(curve_reversed.point_at_start(), curve.point_at_end()));

        // Swap coordinates axes
        curve.swap_coordinates(0, 1);
        // MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(0), Point(0.0, 0.0, 0.0)));
        // MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(1), Point(2.0, 1.0, 0.0)));
        // MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(2), Point(0.0, 2.0, 0.0)));
        // MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(3), Point(2.0, 3.0, 0.0)));
        // MINI_CHECK(TOLERANCE.is_point_close(curve.get_cv(4), Point(0.0, 4.0, 0.0)));

        // Split curve
        NurbsCurve curve_left;
        NurbsCurve curve_right;
        curve.split(0.5, curve_left, curve_right);
        // MINI_CHECK(TOLERANCE.is_point_close(curve.point_at_middle(), curve_left.point_at_end()));
        // MINI_CHECK(TOLERANCE.is_point_close(curve.point_at_middle(), curve_right.point_at_start()));

        // Extend curve
        NurbsCurve curve_extended = curve;
        curve_extended.extend(0.1, 0.1);
        // MINI_CHECK(curve_extended.length() > curve.length());

        // Make rational or non-rational
        curve.make_rational();
        // MINI_CHECK(curve.is_rational() == true);
        for(size_t i = 0; i < static_cast<size_t>(curve.cv_count()); ++i){
            // MINI_CHECK(TOLERANCE.is_close(curve.weight(static_cast<int>(i)), 1.0));
        }
        
        curve.make_non_rational();
        // MINI_CHECK(curve.is_rational() == false);

        // Clamp ends - create unclamped curve manually
        std::vector<Point> points_open = points;
        NurbsCurve curve_open(3, false, 3, 5);  // dim=3, non-rational, order=3 (deg 2), 5 CVs

        // Set control points
        for (int i = 0; i < 5; ++i)
            curve_open.set_cv(i, points_open[i]);

        // Set unclamped uniform knots: {0, 1, 2, 3, 4, 5} (knot_count = order + cv_count - 2 = 6)
        for (int i = 0; i < curve_open.knot_count(); ++i)
            curve_open.set_knot(i, static_cast<double>(i));

        // Now clamp
        curve_open.clamp_end(2);  // 2 = both ends

        std::vector<double> knots = curve_open.get_knots();
        // MINI_CHECK(TOLERANCE.is_close(knots[0], knots[1]));
        // MINI_CHECK(TOLERANCE.is_close(knots[knots.size() - 2], knots[knots.size() - 1]));

    return 0;
}
