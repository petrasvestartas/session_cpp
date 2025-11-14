#include "src/nurbscurve.hpp"
#include <iostream>

int main() {
    using namespace session_cpp;
    
    std::vector<Point> points = {
        Point(0, 0, 0),
        Point(1, 1, 0),
        Point(2, 0, 0),
        Point(3, 1, 0),
        Point(4, 0, 0),
        Point(5, 1, 0)
    };
    
    auto curve = NurbsCurve::create(false, 3, points);
    
    std::cout << "CVs: " << curve.cv_count() << ", Order: " << curve.order() << std::endl;
    std::cout << "Knot count: " << curve.knot_count() << std::endl;
    std::cout << "Knots: ";
    for (int i = 0; i < curve.knot_count(); i++) {
        std::cout << curve.knot(i) << " ";
    }
    std::cout << std::endl;
    
    try {
        auto p_start = curve.point_at_start();
        std::cout << "Start: (" << p_start.x << ", " << p_start.y << ", " << p_start.z << ")" << std::endl;
        
        double len = curve.length();
        std::cout << "Length: " << len << std::endl;
        
        std::cout << "Success!" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    
    return 0;
}
