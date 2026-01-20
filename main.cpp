#include "src/nurbscurve.h"
#include "src/point.h"
#include <iostream>

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

    // Split test
    NurbsCurve left, right;
    double t = curve.domain_middle();
    curve.split(t, left, right);

    std::vector<Point> divisions;
    std::vector<double> params;
    curve.divide_by_count(5, divisions, &params, true);
    std::cout<< "curve" << std::endl;
    for (const auto& p : divisions) {
        std::cout << p << std::endl;
    }

    left.divide_by_count(5, divisions, &params, true);
    std::cout<< "left" << std::endl;
    for (const auto& p : divisions) {
        std::cout << p << std::endl;
    }
    
    right.divide_by_count(5, divisions, &params, true);

    std::cout<< "right" << std::endl;
    for (const auto& p : divisions) {
        std::cout << p << std::endl;
    }

    return 0;
}
