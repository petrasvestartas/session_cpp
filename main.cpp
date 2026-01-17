#include "src/nurbscurve.h"
#include "src/point.h"
#include "src/vector.h"
#include <iostream>
#include <iomanip>

using namespace session_cpp;

int main() {
    // S-wave test curve: degree 2 NURBS with 5 control points
    std::vector<Point> points = {
        Point(0, 0, 0),
        Point(1, 2, 0),
        Point(2, 0, 0),
        Point(3, 2, 0),
        Point(4, 0, 0)
    };

    NurbsCurve curve = NurbsCurve::create(false, 2, points);

    std::cout << std::fixed << std::setprecision(15);

    // divide_by_count(10) with include_endpoints=true should give 10 points (9 segments)
    std::cout << "=== divide_by_count(10, ..., true) ===" << std::endl;
    std::vector<Point> div_pts;
    std::vector<double> div_params;
    curve.divide_by_count(10, div_pts, &div_params, true);

    std::cout << "MINI_CHECK(div_pts.size() == " << div_pts.size() << ");" << std::endl;
    for (size_t i = 0; i < div_pts.size(); i++) {
        std::cout << "MINI_CHECK(TOLERANCE.is_point_close(div_pts[" << i << "], Point("
                  << div_pts[i][0] << ", " << div_pts[i][1] << ", " << div_pts[i][2] << ")));" << std::endl;
    }

    // divide_by_length(0.5)
    std::cout << "\n=== divide_by_length(0.5) ===" << std::endl;
    std::vector<Point> len_pts;
    std::vector<double> len_params;
    curve.divide_by_length(0.5, len_pts, &len_params);

    std::cout << "MINI_CHECK(len_pts.size() == " << len_pts.size() << ");" << std::endl;
    for (size_t i = 0; i < len_pts.size(); i++) {
        std::cout << "MINI_CHECK(TOLERANCE.is_point_close(len_pts[" << i << "], Point("
                  << len_pts[i][0] << ", " << len_pts[i][1] << ", " << len_pts[i][2] << ")));" << std::endl;
    }

    return 0;
}
