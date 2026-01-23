#include "src/nurbscurve.h"
#include "src/point.h"
#include "src/xform.h"
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

    std::cout << "Before translation" << std::endl;
    std::vector<Point> divided;
    curve.divide_by_count(20, divided);
    for (const auto& p : divided) {
        std::cout << p << std::endl;
    }


    // Translate
    Xform translation = Xform::translation(0.0, 0.0, 10.0);
    curve.transform(translation);

    std::cout << "After translation" << std::endl;
    curve.divide_by_count(20, divided);
    for (const auto& p : divided) {
        std::cout << p << std::endl;
    }
    return 0;
}
