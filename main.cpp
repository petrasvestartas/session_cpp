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

    
    auto curve = NurbsCurve::create(false, 2, points);
    curve.make_rational();
    std::vector<Point> divided_points;


    std::cout << " " << std::endl;
    for (const auto& pt : divided_points) 
        std::cout << pt << std::endl;

    std::cout << " " << std::endl;
    curve.set_weight(2, 1.0);
    curve.divide_by_count(20, divided_points);

    for (const auto& pt : divided_points) 
        std::cout << pt << std::endl;

    std::cout << " " << std::endl;
    curve.set_weight(2, 5.0);
    curve.divide_by_count(20, divided_points);

    for (const auto& pt : divided_points) 
        std::cout << pt << std::endl;

    std::cout << " " << std::endl;
    curve.set_weight(2, 10.0);
    curve.divide_by_count(20, divided_points);

    for (const auto& pt : divided_points) 
        std::cout << pt << std::endl;


    curve.make_non_rational(true);  // force=true, sets all weights to 1.0
    std::cout << " " << std::endl;
    curve.divide_by_count(20, divided_points);

    for (const auto& pt : divided_points)
        std::cout << pt << std::endl;

    return 0;
}
