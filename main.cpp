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
    auto [t0, t1] = curve.domain();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Domain: [" << t0 << ", " << t1 << "]" << std::endl;

    // Check tangent_at at domain boundaries
    std::cout << "\ntangent_at(t0): " << curve.tangent_at(t0) << std::endl;
    std::cout << "tangent_at(t1): " << curve.tangent_at(t1) << std::endl;

    // Check evaluate derivatives at boundaries
    std::cout << "\nevaluate at t0:" << std::endl;
    auto d0 = curve.evaluate(t0, 2);
    for (size_t i = 0; i < d0.size(); ++i) {
        std::cout << "  D" << i << ": " << d0[i] << std::endl;
    }

    std::cout << "\nevaluate at t1:" << std::endl;
    auto d1 = curve.evaluate(t1, 2);
    for (size_t i = 0; i < d1.size(); ++i) {
        std::cout << "  D" << i << ": " << d1[i] << std::endl;
    }

    // Check perpendicular_frame_at
    Point o; Vector x, y, z;
    std::cout << "\nperpendicular_frame_at(0.0, normalized=true):" << std::endl;
    curve.perpendicular_frame_at(0.0, true, o, x, y, z);
    std::cout << "  origin: " << o << std::endl;
    std::cout << "  xaxis:  " << x << std::endl;
    std::cout << "  yaxis:  " << y << std::endl;
    std::cout << "  zaxis:  " << z << std::endl;

    std::cout << "\nperpendicular_frame_at(0.5, normalized=true):" << std::endl;
    curve.perpendicular_frame_at(0.5, true, o, x, y, z);
    std::cout << "  origin: " << o << std::endl;
    std::cout << "  xaxis:  " << x << std::endl;
    std::cout << "  yaxis:  " << y << std::endl;
    std::cout << "  zaxis:  " << z << std::endl;

    std::cout << "\nperpendicular_frame_at(1.0, normalized=true):" << std::endl;
    curve.perpendicular_frame_at(1.0, true, o, x, y, z);
    std::cout << "  origin: " << o << std::endl;
    std::cout << "  xaxis:  " << x << std::endl;
    std::cout << "  yaxis:  " << y << std::endl;
    std::cout << "  zaxis:  " << z << std::endl;

    std::cout << "\nRhino expected at 0.0:" << std::endl;
    std::cout << "  origin: {1.957614, 1.140253, -0.191281}" << std::endl;
    std::cout << "  xaxis:  {0.532768, 0.809399, -0.247046}" << std::endl;
    std::cout << "  yaxis:  {-0.261214, -0.120387, -0.957744}" << std::endl;
    std::cout << "  zaxis:  {-0.804938, 0.574787, 0.147288}" << std::endl;

    return 0;
}
