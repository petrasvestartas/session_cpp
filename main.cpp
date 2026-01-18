#include "src/nurbscurve.h"
#include "src/point.h"
#include "src/vector.h"
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

    // Get point at parameter t
    Point point_at = curve.point_at(0.5);
    std::cout << point_at << std::endl;

    // Get point and derivatives at parameter t
    std::vector<Vector> derivatives = curve.evaluate(0.5, 2);
    for (size_t i = 0; i < derivatives.size(); ++i) {
        Vector d = derivatives[i];
        std::cout << "Derivative " << i << ": " << d << std::endl;
        // d is the i-th derivative at t=0.5
    }

    // Tangent vector at parameter t
    Vector tangent = curve.tangent_at(0.5);
    std::cout << "Tangent at t=0.5: " << tangent << std::endl;
    // normalized=true (default): t in [0,1] mapped to domain
    Point o;
    Vector t, n, b;
    curve.frame_at(0.5, true, o, t, n, b);
    std::cout << "Frame at t=0.5:" << std::endl;
    std::cout << "Origin: " << o << std::endl;
    std::cout << "Tangent: " << t << std::endl;
    std::cout << "Normal: " << n << std::endl;
    std::cout << "Binormal: " << b << std::endl;


    // Perpendicular frame at 
    curve.perpendicular_frame_at(0.5, true, o, t, n, b);
    std::cout << "Perpendicular Frame at t=0.5:" << std::endl;
    std::cout << "Origin: " << o << std::endl;
    std::cout << "Tangent: " << t << std::endl;
    std::cout << "Normal: " << n << std::endl;
    std::cout << "Binormal: " << b << std::endl;

    // Points
    Point p0 = curve.point_at_start();
    Point p1 = curve.point_at_middle();
    Point p2 = curve.point_at_end();
    std::cout << "Start Point: " << p0 << std::endl;
    std::cout << "Middle Point: " << p1 << std::endl;
    std::cout << "End Point: " << p2 << std::endl;

    curve.set_start_point(Point(1.957614, 1.140253, 2.0));
    curve.set_end_point(Point(2.15032, 1.868606, 2.0));
    std::cout << "Modified Start Point: " << curve.point_at_start() << std::endl;
    std::cout << "Modified End Point: " << curve.point_at_end() << std::endl;

    return 0;
}
