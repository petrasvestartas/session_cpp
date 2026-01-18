#include "src/nurbscurve.h"
#include "src/point.h"
#include <iostream>
#include <iomanip>

using namespace session_cpp;

int main() {
    std::vector<Point> points = {
        Point(1.957614, 1.140253, -0.191281),
        Point(0.912252, 1.886721, 0),
        Point(3.089381, 2.701879, -0.696251),
        Point(5.015145, 1.189141, 0.35799),
        Point(1.854155, 0.514663, 0.347694),
        Point(3.309532, 1.328666, 0),
        Point(3.544072, 2.194233, 0.696217),
        Point(2.903513, 2.091287, 0.696217),
        Point(2.752484, 1.45432, 0),
        Point(2.406227, 1.288248, 0),
        Point(2.15032, 1.868606, 0)
    };
    auto curve = NurbsCurve::create(false, 2, points);
    
    std::cout << std::setprecision(6) << std::fixed;
    std::cout << "Degree: " << curve.degree() << std::endl;
    std::cout << "Order: " << curve.order() << std::endl;
    std::cout << "CV count: " << curve.cv_count() << std::endl;
    std::cout << "Domain: [" << curve.domain_start() << ", " << curve.domain_end() << "]" << std::endl;
    
    // Evaluate at raw t=0.5
    std::cout << "\nAt raw t=0.5:" << std::endl;
    auto d1 = curve.evaluate(0.5, 2);
    for (size_t i = 0; i < d1.size(); ++i) {
        std::cout << "  D" << i << ": " << d1[i] << std::endl;
    }
    
    // Evaluate at normalized t=0.5 (middle of curve)
    double t_mid = (curve.domain_start() + curve.domain_end()) / 2.0;
    std::cout << "\nAt normalized t=0.5 (actual t=" << t_mid << "):" << std::endl;
    auto d2 = curve.evaluate(t_mid, 2);
    for (size_t i = 0; i < d2.size(); ++i) {
        std::cout << "  D" << i << ": " << d2[i] << std::endl;
    }
    return 0;
}
