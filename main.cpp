// Build + run: ./bash/run_cpp_main.sh
// Build + run tests: ./bash/test_cpp.sh

#include "nurbscurve.h"
#include "point.h"
#include "vector.h"
#include <iostream>

using namespace session_cpp;

int main() {
    std::vector<Point> points = {
        Point(0.0, 0.0, 0.0),
        Point(1.0, 1.0, 0.0),
        Point(2.0, 0.0, 0.0)
    };

    NurbsCurve curve = NurbsCurve::create(false, 2, points);

    std::cout << "str:  " << curve.str() << std::endl;
    std::cout << "repr: " << curve.repr() << std::endl;

    std::vector<Point> divided;
    curve.divide_by_count(10, divided);

    std::cout << "Subdivided points (" << divided.size() << "):" << std::endl;
    for (const auto& p : divided) {
        std::cout << "  " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    }

    return 0;
}
